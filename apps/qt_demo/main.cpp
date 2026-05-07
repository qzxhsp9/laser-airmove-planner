#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QProcess>
#include <QPushButton>
#include <QSplitter>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTime>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

QString defaultConfigPath() {
    const QString candidate = QDir::current().absoluteFilePath("examples/simple_box_config.json");
    return QFileInfo::exists(candidate) ? candidate : QString();
}

QString defaultOutputPath() {
    return QDir::current().absoluteFilePath("airmove_qt_output");
}

QString yesNo(bool value) {
    return value ? "yes" : "no";
}

struct RenderBox {
    airmove::Vec3 center;
    airmove::Vec3 size;
};

struct RenderSphere {
    airmove::Vec3 center;
    double radius{1.0};
};

struct RenderCylinder {
    airmove::Vec3 center;
    double radius{1.0};
    double height{1.0};
};

class SceneViewWidget final : public QWidget {
public:
    explicit SceneViewWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumSize(620, 440);
        setMouseTracking(true);
        resetView();
    }

    void setScene(const airmove::PlanningProblem& problem, const airmove::PlanningResult& result) {
        raw_path_ = result.raw_path;
        shortcut_path_ = result.shortcut_path;
        smoothed_path_ = result.smoothed_path;

        boxes_.clear();
        spheres_.clear();
        cylinders_.clear();
        for (const auto& box : problem.box_obstacles) {
            boxes_.push_back(RenderBox{box.center, box.size});
        }
        for (const auto& sphere : problem.sphere_obstacles) {
            spheres_.push_back(RenderSphere{sphere.center, sphere.radius});
        }
        for (const auto& cylinder : problem.cylinder_obstacles) {
            cylinders_.push_back(RenderCylinder{cylinder.center, cylinder.radius, cylinder.height});
        }

        workspace_min_ = problem.planner_config.workspace_min;
        workspace_max_ = problem.planner_config.workspace_max;
        updateSceneBounds();
        update();
    }

    void resetView() {
        yaw_ = -35.0 * kPi / 180.0;
        pitch_ = 25.0 * kPi / 180.0;
        zoom_ = 1.0;
        pan_ = QPointF(0.0, 0.0);
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor(248, 249, 251));

        drawTitle(painter);
        drawWorkspace(painter);
        drawObstacles(painter);
        drawPath(painter, raw_path_, QColor(115, 115, 115), 1.3, Qt::SolidLine);
        drawPath(painter, shortcut_path_, QColor(230, 159, 0), 1.8, Qt::DashLine);
        drawPath(painter, smoothed_path_, QColor(0, 114, 178), 2.5, Qt::SolidLine);
        drawAxes(painter);
        drawLegend(painter);
    }

    void mousePressEvent(QMouseEvent* event) override {
        last_mouse_ = event->pos();
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        const QPoint delta = event->pos() - last_mouse_;
        last_mouse_ = event->pos();
        if (event->buttons() & Qt::LeftButton) {
            yaw_ += static_cast<double>(delta.x()) * 0.008;
            pitch_ += static_cast<double>(delta.y()) * 0.008;
            pitch_ = std::clamp(pitch_, -1.45, 1.45);
            update();
        } else if (event->buttons() & Qt::RightButton) {
            pan_ += QPointF(delta);
            update();
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
        zoom_ *= std::pow(1.12, steps);
        zoom_ = std::clamp(zoom_, 0.15, 8.0);
        update();
    }

private:
    void updateSceneBounds() {
        bounds_min_ = workspace_min_;
        bounds_max_ = workspace_max_;

        const auto includePoint = [this](const airmove::Vec3& point) {
            bounds_min_ = bounds_min_.cwiseMin(point);
            bounds_max_ = bounds_max_.cwiseMax(point);
        };

        for (const auto& p : raw_path_) {
            includePoint(p);
        }
        for (const auto& p : shortcut_path_) {
            includePoint(p);
        }
        for (const auto& p : smoothed_path_) {
            includePoint(p);
        }
        for (const auto& box : boxes_) {
            includePoint(box.center - box.size * 0.5);
            includePoint(box.center + box.size * 0.5);
        }
        for (const auto& sphere : spheres_) {
            includePoint(sphere.center - airmove::Vec3::Constant(sphere.radius));
            includePoint(sphere.center + airmove::Vec3::Constant(sphere.radius));
        }
        for (const auto& cylinder : cylinders_) {
            includePoint(cylinder.center - airmove::Vec3(cylinder.radius, cylinder.radius, cylinder.height * 0.5));
            includePoint(cylinder.center + airmove::Vec3(cylinder.radius, cylinder.radius, cylinder.height * 0.5));
        }

        scene_center_ = 0.5 * (bounds_min_ + bounds_max_);
        scene_radius_ = std::max(1.0, 0.5 * (bounds_max_ - bounds_min_).norm());
    }

    QPointF project(const airmove::Vec3& point) const {
        const airmove::Vec3 p = point - scene_center_;

        const double cy = std::cos(yaw_);
        const double sy = std::sin(yaw_);
        const double cp = std::cos(pitch_);
        const double sp = std::sin(pitch_);

        const double x1 = cy * p.x() - sy * p.y();
        const double y1 = sy * p.x() + cy * p.y();
        const double z1 = p.z();

        const double y2 = cp * y1 - sp * z1;
        const double scale = zoom_ * std::min(width(), height()) / (2.4 * scene_radius_);
        return QPointF(width() * 0.5 + pan_.x() + x1 * scale,
                       height() * 0.52 + pan_.y() - y2 * scale);
    }

    void drawTitle(QPainter& painter) {
        painter.setPen(QColor(45, 45, 45));
        painter.setFont(QFont("Segoe UI", 11, QFont::DemiBold));
        painter.drawText(18, 26, "Interactive 3D Path View");
        painter.setFont(QFont("Segoe UI", 8));
        painter.setPen(QColor(95, 95, 95));
        painter.drawText(18, 46, "Left drag: rotate    Right drag: pan    Wheel: zoom");
    }

    void drawLine3(QPainter& painter, const airmove::Vec3& a, const airmove::Vec3& b) {
        painter.drawLine(project(a), project(b));
    }

    void drawWorkspace(QPainter& painter) {
        painter.setPen(QPen(QColor(160, 165, 172), 1.0, Qt::DotLine));
        drawBoxWire(painter, scene_center_, bounds_max_ - bounds_min_);
    }

    void drawObstacles(QPainter& painter) {
        painter.setPen(QPen(QColor(140, 60, 0), 1.2));
        painter.setBrush(QColor(213, 94, 0, 38));
        for (const auto& box : boxes_) {
            drawBoxWire(painter, box.center, box.size);
        }

        painter.setPen(QPen(QColor(170, 80, 0), 1.1));
        for (const auto& sphere : spheres_) {
            drawCircle3(painter, sphere.center, sphere.radius, 0);
            drawCircle3(painter, sphere.center, sphere.radius, 1);
            drawCircle3(painter, sphere.center, sphere.radius, 2);
        }

        painter.setPen(QPen(QColor(180, 90, 0), 1.1));
        for (const auto& cylinder : cylinders_) {
            drawCylinderWire(painter, cylinder);
        }
    }

    void drawPath(QPainter& painter,
                  const airmove::Path3& path,
                  const QColor& color,
                  double width,
                  Qt::PenStyle style) {
        if (path.size() < 2) {
            return;
        }
        painter.setPen(QPen(color, width, style, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(Qt::NoBrush);
        QPainterPath qpath;
        qpath.moveTo(project(path.front()));
        for (std::size_t i = 1; i < path.size(); ++i) {
            qpath.lineTo(project(path[i]));
        }
        painter.drawPath(qpath);

        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(project(path.front()), 4.5, 4.5);
        painter.drawEllipse(project(path.back()), 4.5, 4.5);
    }

    void drawAxes(QPainter& painter) {
        const double len = scene_radius_ * 0.22;
        const airmove::Vec3 origin = bounds_min_ + airmove::Vec3(0.08, 0.08, 0.08).cwiseProduct(bounds_max_ - bounds_min_);
        painter.setFont(QFont("Segoe UI", 8, QFont::DemiBold));

        painter.setPen(QPen(QColor(200, 55, 55), 2.0));
        drawLine3(painter, origin, origin + airmove::Vec3(len, 0.0, 0.0));
        painter.drawText(project(origin + airmove::Vec3(len, 0.0, 0.0)), "X");

        painter.setPen(QPen(QColor(35, 150, 85), 2.0));
        drawLine3(painter, origin, origin + airmove::Vec3(0.0, len, 0.0));
        painter.drawText(project(origin + airmove::Vec3(0.0, len, 0.0)), "Y");

        painter.setPen(QPen(QColor(45, 90, 200), 2.0));
        drawLine3(painter, origin, origin + airmove::Vec3(0.0, 0.0, len));
        painter.drawText(project(origin + airmove::Vec3(0.0, 0.0, len)), "Z");
    }

    void drawLegend(QPainter& painter) {
        const int x = width() - 185;
        int y = 22;
        auto item = [&](const QColor& color, Qt::PenStyle style, const QString& text) {
            painter.setPen(QPen(color, 2.5, style));
            painter.drawLine(x, y, x + 30, y);
            painter.setPen(QColor(55, 55, 55));
            painter.drawText(x + 38, y + 5, text);
            y += 22;
        };
        painter.setFont(QFont("Segoe UI", 8));
        item(QColor(115, 115, 115), Qt::SolidLine, "raw");
        item(QColor(230, 159, 0), Qt::DashLine, "shortcut");
        item(QColor(0, 114, 178), Qt::SolidLine, "smoothed");
        item(QColor(213, 94, 0), Qt::SolidLine, "obstacle");
    }

    void drawBoxWire(QPainter& painter, const airmove::Vec3& center, const airmove::Vec3& size) {
        const airmove::Vec3 h = size * 0.5;
        const std::array<airmove::Vec3, 8> v{
            center + airmove::Vec3(-h.x(), -h.y(), -h.z()),
            center + airmove::Vec3(h.x(), -h.y(), -h.z()),
            center + airmove::Vec3(h.x(), h.y(), -h.z()),
            center + airmove::Vec3(-h.x(), h.y(), -h.z()),
            center + airmove::Vec3(-h.x(), -h.y(), h.z()),
            center + airmove::Vec3(h.x(), -h.y(), h.z()),
            center + airmove::Vec3(h.x(), h.y(), h.z()),
            center + airmove::Vec3(-h.x(), h.y(), h.z()),
        };
        const int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
            {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
        };
        for (const auto& edge : edges) {
            drawLine3(painter, v[edge[0]], v[edge[1]]);
        }
    }

    void drawCircle3(QPainter& painter, const airmove::Vec3& center, double radius, int plane) {
        airmove::Vec3 prev;
        constexpr int segments = 48;
        for (int i = 0; i <= segments; ++i) {
            const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(segments);
            airmove::Vec3 p = center;
            if (plane == 0) {
                p += airmove::Vec3(radius * std::cos(a), radius * std::sin(a), 0.0);
            } else if (plane == 1) {
                p += airmove::Vec3(radius * std::cos(a), 0.0, radius * std::sin(a));
            } else {
                p += airmove::Vec3(0.0, radius * std::cos(a), radius * std::sin(a));
            }
            if (i > 0) {
                drawLine3(painter, prev, p);
            }
            prev = p;
        }
    }

    void drawCylinderWire(QPainter& painter, const RenderCylinder& cylinder) {
        constexpr int segments = 36;
        const double z0 = cylinder.center.z() - cylinder.height * 0.5;
        const double z1 = cylinder.center.z() + cylinder.height * 0.5;
        airmove::Vec3 prev0;
        airmove::Vec3 prev1;
        for (int i = 0; i <= segments; ++i) {
            const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(segments);
            const double x = cylinder.center.x() + cylinder.radius * std::cos(a);
            const double y = cylinder.center.y() + cylinder.radius * std::sin(a);
            const airmove::Vec3 p0(x, y, z0);
            const airmove::Vec3 p1(x, y, z1);
            if (i > 0) {
                drawLine3(painter, prev0, p0);
                drawLine3(painter, prev1, p1);
            }
            if (i % 9 == 0) {
                drawLine3(painter, p0, p1);
            }
            prev0 = p0;
            prev1 = p1;
        }
    }

    airmove::Path3 raw_path_;
    airmove::Path3 shortcut_path_;
    airmove::Path3 smoothed_path_;
    std::vector<RenderBox> boxes_;
    std::vector<RenderSphere> spheres_;
    std::vector<RenderCylinder> cylinders_;
    airmove::Vec3 workspace_min_{-100.0, -100.0, 0.0};
    airmove::Vec3 workspace_max_{300.0, 300.0, 200.0};
    airmove::Vec3 bounds_min_{-100.0, -100.0, 0.0};
    airmove::Vec3 bounds_max_{300.0, 300.0, 200.0};
    airmove::Vec3 scene_center_{100.0, 100.0, 100.0};
    double scene_radius_{250.0};
    double yaw_{0.0};
    double pitch_{0.0};
    double zoom_{1.0};
    QPointF pan_{0.0, 0.0};
    QPoint last_mouse_;
};

class MainWindow final : public QMainWindow {
public:
    MainWindow() {
        setWindowTitle("Laser Air-Move Planner Qt Demo");
        resize(1320, 860);
        buildUi();
        connectUi();
        config_path_->setText(defaultConfigPath());
        output_path_->setText(defaultOutputPath());
        loadConfigPreview();
    }

private:
    void buildUi() {
        auto* root = new QSplitter(Qt::Horizontal, this);
        root->setChildrenCollapsible(false);
        setCentralWidget(root);

        auto* controls = new QWidget(root);
        controls->setMinimumWidth(330);
        controls->setMaximumWidth(420);
        auto* controls_layout = new QVBoxLayout(controls);

        auto* io_group = new QGroupBox("Input / Output", controls);
        auto* io_layout = new QFormLayout(io_group);
        config_path_ = new QLineEdit(io_group);
        output_path_ = new QLineEdit(io_group);
        auto* browse_config = new QPushButton("Choose config", io_group);
        auto* browse_output = new QPushButton("Choose output directory", io_group);
        io_layout->addRow("Config", config_path_);
        io_layout->addRow("", browse_config);
        io_layout->addRow("Output", output_path_);
        io_layout->addRow("", browse_output);
        controls_layout->addWidget(io_group);

        auto* params_group = new QGroupBox("Planner Params", controls);
        auto* params_layout = new QFormLayout(params_group);
        planner_type_ = new QComboBox(params_group);
        planner_type_->addItems({"informed_rrtstar", "rrtconnect", "rrtstar"});
        planning_time_ = new QDoubleSpinBox(params_group);
        planning_time_->setRange(0.05, 30.0);
        planning_time_->setDecimals(2);
        planning_time_->setSingleStep(0.25);
        validity_resolution_ = new QDoubleSpinBox(params_group);
        validity_resolution_->setRange(0.1, 50.0);
        validity_resolution_->setDecimals(2);
        validity_resolution_->setSingleStep(0.5);
        smoothing_samples_ = new QSpinBox(params_group);
        smoothing_samples_->setRange(2, 1000);
        safety_margin_ = new QDoubleSpinBox(params_group);
        safety_margin_->setRange(0.0, 100.0);
        safety_margin_->setDecimals(2);
        simplify_solution_ = new QCheckBox("Enable OMPL simplifySolution", params_group);

        params_layout->addRow("planner", planner_type_);
        params_layout->addRow("planning_time", planning_time_);
        params_layout->addRow("validity_resolution", validity_resolution_);
        params_layout->addRow("smoothing_samples", smoothing_samples_);
        params_layout->addRow("safety_margin", safety_margin_);
        params_layout->addRow("", simplify_solution_);
        controls_layout->addWidget(params_group);

        auto* actions_group = new QGroupBox("Actions", controls);
        auto* actions_layout = new QVBoxLayout(actions_group);
        run_plan_ = new QPushButton("Run planning", actions_group);
        run_visualization_ = new QPushButton("Export PNG visualization", actions_group);
        reset_view_ = new QPushButton("Reset 3D view", actions_group);
        load_preview_ = new QPushButton("Reload config", actions_group);
        actions_layout->addWidget(run_plan_);
        actions_layout->addWidget(run_visualization_);
        actions_layout->addWidget(reset_view_);
        actions_layout->addWidget(load_preview_);
        controls_layout->addWidget(actions_group);

        auto* metrics_group = new QGroupBox("Summary", controls);
        auto* metrics_layout = new QFormLayout(metrics_group);
        success_label_ = new QLabel("-", metrics_group);
        raw_label_ = new QLabel("-", metrics_group);
        shortcut_label_ = new QLabel("-", metrics_group);
        smooth_label_ = new QLabel("-", metrics_group);
        duration_label_ = new QLabel("-", metrics_group);
        clearance_label_ = new QLabel("-", metrics_group);
        smoothing_label_ = new QLabel("-", metrics_group);
        metrics_layout->addRow("success", success_label_);
        metrics_layout->addRow("raw length", raw_label_);
        metrics_layout->addRow("shortcut length", shortcut_label_);
        metrics_layout->addRow("smooth length", smooth_label_);
        metrics_layout->addRow("trajectory duration", duration_label_);
        metrics_layout->addRow("min clearance", clearance_label_);
        metrics_layout->addRow("smoothing", smoothing_label_);
        controls_layout->addWidget(metrics_group);
        controls_layout->addStretch(1);

        connect(browse_config, &QPushButton::clicked, this, [this]() {
            const QString path = QFileDialog::getOpenFileName(
                this, "Choose planner config", QDir::currentPath(), "JSON (*.json)");
            if (!path.isEmpty()) {
                config_path_->setText(path);
                loadConfigPreview();
            }
        });
        connect(browse_output, &QPushButton::clicked, this, [this]() {
            const QString path = QFileDialog::getExistingDirectory(this, "Choose output directory", output_path_->text());
            if (!path.isEmpty()) {
                output_path_->setText(path);
            }
        });

        tabs_ = new QTabWidget(root);
        scene_view_ = new SceneViewWidget(tabs_);
        tabs_->addTab(scene_view_, "Interactive 3D View");

        path_table_ = new QTableWidget(tabs_);
        path_table_->setColumnCount(4);
        path_table_->setHorizontalHeaderLabels({"index", "x", "y", "z"});
        path_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tabs_->addTab(path_table_, "smoothed path");

        config_preview_ = new QTextEdit(tabs_);
        config_preview_->setReadOnly(true);
        tabs_->addTab(config_preview_, "Config preview");

        log_ = new QTextEdit(tabs_);
        log_->setReadOnly(true);
        tabs_->addTab(log_, "Log");

        root->addWidget(controls);
        root->addWidget(tabs_);
        root->setStretchFactor(1, 1);

        statusBar()->showMessage("Ready");
    }

    void connectUi() {
        connect(load_preview_, &QPushButton::clicked, this, [this]() { loadConfigPreview(); });
        connect(run_plan_, &QPushButton::clicked, this, [this]() { runPlanning(); });
        connect(run_visualization_, &QPushButton::clicked, this, [this]() { runVisualization(); });
        connect(reset_view_, &QPushButton::clicked, this, [this]() { scene_view_->resetView(); });
    }

    void loadConfigPreview() {
        QFile file(config_path_->text());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            config_preview_->setPlainText("Cannot read config file.");
            return;
        }
        config_preview_->setPlainText(QString::fromUtf8(file.readAll()));

        try {
            last_problem_ = airmove::loadPlanningProblemJson(config_path_->text().toStdString());
            const auto& cfg = last_problem_.planner_config;
            planner_type_->setCurrentText(QString::fromStdString(cfg.planner_type));
            planning_time_->setValue(cfg.planning_time_limit);
            validity_resolution_->setValue(cfg.validity_resolution);
            smoothing_samples_->setValue(cfg.smoothing_samples);
            safety_margin_->setValue(cfg.safety_margin);
            simplify_solution_->setChecked(cfg.simplify_solution);
            scene_view_->setScene(last_problem_, last_result_);
            appendLog("Config loaded: " + config_path_->text());
        } catch (const std::exception& e) {
            appendLog(QString("Config parse failed: %1").arg(e.what()));
        }
    }

    void runPlanning() {
        setBusy(true);
        try {
            last_problem_ = airmove::loadPlanningProblemJson(config_path_->text().toStdString());
            last_problem_.planner_config.planner_type = planner_type_->currentText().toStdString();
            last_problem_.planner_config.planning_time_limit = planning_time_->value();
            last_problem_.request.planning_time = planning_time_->value();
            last_problem_.planner_config.validity_resolution = validity_resolution_->value();
            last_problem_.planner_config.smoothing_samples = smoothing_samples_->value();
            last_problem_.planner_config.safety_margin = safety_margin_->value();
            last_problem_.request.safety_margin = safety_margin_->value();
            last_problem_.planner_config.simplify_solution = simplify_solution_->isChecked();

            auto world = airmove::buildCollisionWorld(last_problem_);
            airmove::AirMovePlanner planner(last_problem_.planner_config);
            const auto begin = QDateTime::currentMSecsSinceEpoch();
            last_result_ = planner.plan(last_problem_.request, world);
            const auto elapsed = QDateTime::currentMSecsSinceEpoch() - begin;

            airmove::writePlanningOutputs(output_path_->text().toStdString(), last_result_);
            updateSummary(last_result_);
            updatePathTable(last_result_.smoothed_path);
            scene_view_->setScene(last_problem_, last_result_);
            appendLog(QString("Planning finished in %1 ms: %2")
                          .arg(elapsed)
                          .arg(QString::fromStdString(last_result_.message)));
            statusBar()->showMessage(last_result_.success ? "Planning succeeded" : "Planning failed");
            tabs_->setCurrentWidget(scene_view_);
        } catch (const std::exception& e) {
            appendLog(QString("Planning exception: %1").arg(e.what()));
            QMessageBox::critical(this, "Planning exception", e.what());
        }
        setBusy(false);
    }

    void runVisualization() {
        const QString script = QDir::current().absoluteFilePath("tools/visualize_path.py");
        if (!QFileInfo::exists(script)) {
            QMessageBox::warning(this, "Missing script", "Cannot find tools/visualize_path.py");
            return;
        }

        const QString image_dir = QDir(output_path_->text()).absoluteFilePath("visualization");
        QStringList args{
            "-3.14",
            script,
            "--config",
            config_path_->text(),
            "--input",
            output_path_->text(),
            "--output",
            image_dir,
        };

        appendLog("Exporting PNG visualization...");
        QProcess process;
        process.setProgram("py");
        process.setArguments(args);
        process.setProcessChannelMode(QProcess::MergedChannels);
        process.start();
        if (!process.waitForStarted(3000)) {
            QMessageBox::warning(this, "Start failed", "Cannot start Python. Run tools/visualize_path.py manually.");
            return;
        }
        process.waitForFinished(120000);
        appendLog(QString::fromLocal8Bit(process.readAll()));

        const QString image_path = QDir(image_dir).absoluteFilePath("path_3d.png");
        if (!QFileInfo::exists(image_path)) {
            QMessageBox::warning(this, "Export failed", "path_3d.png was not generated. Check the log.");
            return;
        }
        QMessageBox::information(this, "Export complete", "PNG visualization written to:\n" + image_dir);
        statusBar()->showMessage("PNG visualization generated");
    }

    void updateSummary(const airmove::PlanningResult& result) {
        const double duration = result.trajectory.empty() ? 0.0 : result.trajectory.back().time;
        success_label_->setText(yesNo(result.success));
        raw_label_->setText(QString::number(result.raw_path_length, 'f', 3));
        shortcut_label_->setText(QString::number(result.shortcut_path_length, 'f', 3));
        smooth_label_->setText(QString::number(result.smoothed_path_length, 'f', 3));
        duration_label_->setText(QString::number(duration, 'f', 3));
        clearance_label_->setText(QString::number(result.min_clearance, 'f', 3));
        smoothing_label_->setText(QString("%1, fallback=%2")
                                      .arg(yesNo(result.smoothing_used))
                                      .arg(yesNo(result.smoothing_fallback)));
    }

    void updatePathTable(const airmove::Path3& path) {
        path_table_->setRowCount(static_cast<int>(path.size()));
        for (int row = 0; row < static_cast<int>(path.size()); ++row) {
            path_table_->setItem(row, 0, new QTableWidgetItem(QString::number(row)));
            path_table_->setItem(row, 1, new QTableWidgetItem(QString::number(path[row].x(), 'f', 3)));
            path_table_->setItem(row, 2, new QTableWidgetItem(QString::number(path[row].y(), 'f', 3)));
            path_table_->setItem(row, 3, new QTableWidgetItem(QString::number(path[row].z(), 'f', 3)));
        }
    }

    void appendLog(const QString& message) {
        log_->append(QString("[%1] %2")
                         .arg(QTime::currentTime().toString("HH:mm:ss"))
                         .arg(message.trimmed()));
    }

    void setBusy(bool busy) {
        run_plan_->setEnabled(!busy);
        run_visualization_->setEnabled(!busy);
        reset_view_->setEnabled(!busy);
        load_preview_->setEnabled(!busy);
        if (busy) {
            QApplication::setOverrideCursor(Qt::WaitCursor);
        } else if (QApplication::overrideCursor() != nullptr) {
            QApplication::restoreOverrideCursor();
        }
    }

    QLineEdit* config_path_{nullptr};
    QLineEdit* output_path_{nullptr};
    QComboBox* planner_type_{nullptr};
    QDoubleSpinBox* planning_time_{nullptr};
    QDoubleSpinBox* validity_resolution_{nullptr};
    QDoubleSpinBox* safety_margin_{nullptr};
    QSpinBox* smoothing_samples_{nullptr};
    QCheckBox* simplify_solution_{nullptr};
    QPushButton* run_plan_{nullptr};
    QPushButton* run_visualization_{nullptr};
    QPushButton* reset_view_{nullptr};
    QPushButton* load_preview_{nullptr};
    QLabel* success_label_{nullptr};
    QLabel* raw_label_{nullptr};
    QLabel* shortcut_label_{nullptr};
    QLabel* smooth_label_{nullptr};
    QLabel* duration_label_{nullptr};
    QLabel* clearance_label_{nullptr};
    QLabel* smoothing_label_{nullptr};
    QTabWidget* tabs_{nullptr};
    SceneViewWidget* scene_view_{nullptr};
    QTableWidget* path_table_{nullptr};
    QTextEdit* config_preview_{nullptr};
    QTextEdit* log_{nullptr};
    airmove::PlanningProblem last_problem_;
    airmove::PlanningResult last_result_;
};

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
