#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"
#include "airmove/StlMeshLoader.hpp"

#include <QApplication>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
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
#include <QTableWidgetItem>
#include <QTextEdit>
#include <QTime>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using json = nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinPitch = -75.0 * kPi / 180.0;
constexpr double kMaxPitch = 75.0 * kPi / 180.0;
constexpr double kMinZoom = 0.08;
constexpr double kMaxZoom = 18.0;

constexpr int kObstacleTypeCol = 0;
constexpr int kObstacleXCol = 1;
constexpr int kObstacleYCol = 2;
constexpr int kObstacleZCol = 3;
constexpr int kObstacleACol = 4;
constexpr int kObstacleBCol = 5;
constexpr int kObstacleCCol = 6;

QString findUpwards(const QString& start_dir, const QString& relative_file) {
    QDir dir(start_dir);
    while (true) {
        const QString candidate = dir.absoluteFilePath(relative_file);
        if (QFileInfo::exists(candidate)) {
            return QFileInfo(candidate).absoluteFilePath();
        }
        if (!dir.cdUp()) {
            break;
        }
    }
    return {};
}

QString findProjectFile(const QString& relative_file, const QString& config_path = {}) {
    const QStringList starts{
        QDir::currentPath(),
        QCoreApplication::applicationDirPath(),
        config_path.isEmpty() ? QString() : QFileInfo(config_path).absolutePath(),
    };
    for (const QString& start : starts) {
        if (start.isEmpty()) {
            continue;
        }
        const QString found = findUpwards(start, relative_file);
        if (!found.isEmpty()) {
            return found;
        }
    }
    return {};
}

QString findProjectRoot(const QString& config_path = {}) {
    const QString cmake = findProjectFile("CMakeLists.txt", config_path);
    return cmake.isEmpty() ? QDir::currentPath() : QFileInfo(cmake).absolutePath();
}

QString defaultConfigPath() {
    return findProjectFile("examples/simple_box_config.json");
}

QString defaultOutputPath() {
    return QDir(findProjectRoot()).absoluteFilePath("airmove_qt_output");
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

struct RenderTriangle {
    std::array<airmove::Vec3, 3> vertices;
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
        meshes_.clear();
        for (const auto& box : problem.box_obstacles) {
            boxes_.push_back(RenderBox{box.center, box.size});
        }
        for (const auto& sphere : problem.sphere_obstacles) {
            spheres_.push_back(RenderSphere{sphere.center, sphere.radius});
        }
        for (const auto& cylinder : problem.cylinder_obstacles) {
            cylinders_.push_back(RenderCylinder{cylinder.center, cylinder.radius, cylinder.height});
        }
        for (const auto& mesh : problem.mesh_obstacles) {
            try {
                const auto loaded = airmove::loadAsciiStl(mesh.file);
                for (const auto& triangle : loaded.triangles) {
                    meshes_.push_back(RenderTriangle{{
                        loaded.vertices[static_cast<std::size_t>(triangle.x())],
                        loaded.vertices[static_cast<std::size_t>(triangle.y())],
                        loaded.vertices[static_cast<std::size_t>(triangle.z())],
                    }});
                }
            } catch (const std::exception&) {
                // Keep the scene usable even if an optional mesh preview cannot be loaded.
            }
        }

        workspace_min_ = problem.planner_config.workspace_min;
        workspace_max_ = problem.planner_config.workspace_max;
        start_point_ = problem.request.start.position;
        goal_point_ = problem.request.goal.position;
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
        drawEndpoints(painter);
        drawAxes(painter);
        drawLegend(painter);
    }

    void mousePressEvent(QMouseEvent* event) override {
        last_mouse_ = event->pos();
        if (event->button() == Qt::LeftButton) {
            setCursor(Qt::ClosedHandCursor);
        } else if (event->button() == Qt::RightButton) {
            setCursor(Qt::SizeAllCursor);
        }
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        QWidget::mouseReleaseEvent(event);
        if (event->buttons() == Qt::NoButton) {
            unsetCursor();
        }
    }

    void mouseDoubleClickEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            resetView();
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        const QPoint delta = event->pos() - last_mouse_;
        const QPoint anchor_screen = event->pos();
        const airmove::Vec3 anchor_world = screenToWorldOnViewPlane(anchor_screen);
        last_mouse_ = event->pos();
        if (event->buttons() & Qt::LeftButton) {
            const double base = rotationRadiansPerPixel();
            const double modifier = rotationModifier(event->modifiers());
            yaw_ = normalizeAngle(yaw_ + static_cast<double>(delta.x()) * base * modifier);
            pitch_ = std::clamp(pitch_ + static_cast<double>(delta.y()) * base * 0.82 * modifier,
                                kMinPitch,
                                kMaxPitch);
            keepWorldPointAtScreen(anchor_world, anchor_screen);
            update();
        } else if (event->buttons() & Qt::RightButton) {
            pan_ += QPointF(delta) * 0.92;
            update();
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        const QPoint anchor_screen = event->position().toPoint();
        const airmove::Vec3 anchor_world = screenToWorldOnViewPlane(anchor_screen);
        double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
        if (steps == 0.0 && !event->pixelDelta().isNull()) {
            steps = static_cast<double>(event->pixelDelta().y()) / 90.0;
        }
        steps = std::clamp(steps, -4.0, 4.0);
        zoom_ *= std::pow(1.15, steps);
        zoom_ = std::clamp(zoom_, kMinZoom, kMaxZoom);
        keepWorldPointAtScreen(anchor_world, anchor_screen);
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
        includePoint(start_point_);
        includePoint(goal_point_);
        includePoint(airmove::Vec3::Zero());
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
        for (const auto& triangle : meshes_) {
            for (const auto& vertex : triangle.vertices) {
                includePoint(vertex);
            }
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

    double currentScale() const {
        return zoom_ * std::min(width(), height()) / (2.4 * scene_radius_);
    }

    airmove::Vec3 screenToWorldOnViewPlane(const QPointF& screen) const {
        const double scale = currentScale();
        if (scale <= 0.0) {
            return scene_center_;
        }

        const double x1 = (screen.x() - width() * 0.5 - pan_.x()) / scale;
        const double y2 = -(screen.y() - height() * 0.52 - pan_.y()) / scale;

        const double cy = std::cos(yaw_);
        const double sy = std::sin(yaw_);
        const double cp = std::cos(pitch_);
        const double sp = std::sin(pitch_);

        const double y1 = cp * y2;
        const double z1 = -sp * y2;
        const double x = cy * x1 + sy * y1;
        const double y = -sy * x1 + cy * y1;
        return scene_center_ + airmove::Vec3(x, y, z1);
    }

    void keepWorldPointAtScreen(const airmove::Vec3& world, const QPointF& screen) {
        pan_ += screen - project(world);
    }

    double rotationRadiansPerPixel() const {
        const double reference = static_cast<double>(std::max(260, std::min(width(), height())));
        return kPi / reference;
    }

    static double rotationModifier(Qt::KeyboardModifiers modifiers) {
        if (modifiers & Qt::ShiftModifier) {
            return 0.35;
        }
        if (modifiers & Qt::ControlModifier) {
            return 1.8;
        }
        return 0.72;
    }

    static double normalizeAngle(double angle) {
        while (angle > kPi) {
            angle -= 2.0 * kPi;
        }
        while (angle < -kPi) {
            angle += 2.0 * kPi;
        }
        return angle;
    }

    void drawTitle(QPainter& painter) {
        painter.setPen(QColor(45, 45, 45));
        painter.setFont(QFont("Segoe UI", 11, QFont::DemiBold));
        painter.drawText(18, 26, "Interactive 3D Path View");
        painter.setFont(QFont("Segoe UI", 8));
        painter.setPen(QColor(95, 95, 95));
        painter.drawText(18, 46, "Left drag: rotate    Right drag: pan    Wheel: zoom    Shift: fine    Ctrl: fast    Double click: reset");
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

        painter.setPen(QPen(QColor(185, 80, 0), 1.1));
        painter.setBrush(Qt::NoBrush);
        for (const auto& triangle : meshes_) {
            drawLine3(painter, triangle.vertices[0], triangle.vertices[1]);
            drawLine3(painter, triangle.vertices[1], triangle.vertices[2]);
            drawLine3(painter, triangle.vertices[2], triangle.vertices[0]);
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

    void drawEndpoints(QPainter& painter) {
        auto marker = [&](const airmove::Vec3& point, const QColor& fill, const QString& label) {
            const QPointF p = project(point);
            painter.setPen(QPen(QColor(35, 35, 35), 1.2));
            painter.setBrush(fill);
            painter.drawEllipse(p, 6.0, 6.0);
            painter.setPen(QColor(35, 35, 35));
            painter.setFont(QFont("Segoe UI", 8, QFont::DemiBold));
            painter.drawText(p + QPointF(8.0, -8.0), label);
        };
        marker(start_point_, QColor(0, 145, 90), "Start");
        marker(goal_point_, QColor(210, 60, 50), "Goal");
    }

    void drawAxes(QPainter& painter) {
        const double len = scene_radius_ * 0.22;
        const airmove::Vec3 origin = airmove::Vec3::Zero();
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
        item(QColor(0, 145, 90), Qt::SolidLine, "start");
        item(QColor(210, 60, 50), Qt::SolidLine, "goal");
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
    std::vector<RenderTriangle> meshes_;
    airmove::Vec3 workspace_min_{-100.0, -100.0, 0.0};
    airmove::Vec3 workspace_max_{300.0, 300.0, 200.0};
    airmove::Vec3 start_point_{0.0, 0.0, 0.0};
    airmove::Vec3 goal_point_{0.0, 0.0, 0.0};
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
    QDoubleSpinBox* makeDoubleSpin(double min, double max, double value, double step = 1.0, int decimals = 3) {
        auto* spin = new QDoubleSpinBox(this);
        spin->setRange(min, max);
        spin->setDecimals(decimals);
        spin->setSingleStep(step);
        spin->setValue(value);
        return spin;
    }

    void addVec3Row(QGridLayout* layout, int row, const QString& label, std::array<QDoubleSpinBox*, 3>& fields) {
        layout->addWidget(new QLabel(label), row, 0);
        for (int i = 0; i < 3; ++i) {
            fields[static_cast<std::size_t>(i)] = makeDoubleSpin(-1000000.0, 1000000.0, 0.0);
            layout->addWidget(fields[static_cast<std::size_t>(i)], row, i + 1);
        }
    }

    static airmove::Vec3 readVec3Fields(const std::array<QDoubleSpinBox*, 3>& fields) {
        return airmove::Vec3(fields[0]->value(), fields[1]->value(), fields[2]->value());
    }

    static void writeVec3Fields(const std::array<QDoubleSpinBox*, 3>& fields, const airmove::Vec3& value) {
        fields[0]->setValue(value.x());
        fields[1]->setValue(value.y());
        fields[2]->setValue(value.z());
    }

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
        planning_elapsed_label_ = new QLabel("-", metrics_group);
        duration_label_ = new QLabel("-", metrics_group);
        clearance_label_ = new QLabel("-", metrics_group);
        smoothing_label_ = new QLabel("-", metrics_group);
        metrics_layout->addRow("success", success_label_);
        metrics_layout->addRow("raw length", raw_label_);
        metrics_layout->addRow("shortcut length", shortcut_label_);
        metrics_layout->addRow("smooth length", smooth_label_);
        metrics_layout->addRow("planning time", planning_elapsed_label_);
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

        log_ = new QTextEdit(tabs_);
        log_->setReadOnly(true);
        tabs_->addTab(log_, "Log");

        scene_editor_ = new QWidget(root);
        scene_editor_->setMinimumWidth(470);
        scene_editor_->setMaximumWidth(640);
        auto* builder_layout = new QVBoxLayout(scene_editor_);
        auto* editor_title = new QLabel("Scene editor", scene_editor_);
        editor_title->setFont(QFont("Segoe UI", 12, QFont::DemiBold));
        builder_layout->addWidget(editor_title);

        auto* scene_group = new QGroupBox("Scene geometry", scene_editor_);
        auto* scene_grid = new QGridLayout(scene_group);
        scene_grid->addWidget(new QLabel("X"), 0, 1);
        scene_grid->addWidget(new QLabel("Y"), 0, 2);
        scene_grid->addWidget(new QLabel("Z"), 0, 3);
        addVec3Row(scene_grid, 1, "workspace min", workspace_min_fields_);
        addVec3Row(scene_grid, 2, "workspace max", workspace_max_fields_);
        addVec3Row(scene_grid, 3, "start xyz", start_fields_);
        addVec3Row(scene_grid, 4, "goal xyz", goal_fields_);
        head_radius_field_ = makeDoubleSpin(0.1, 10000.0, 8.0, 0.5);
        scene_grid->addWidget(new QLabel("tool head radius"), 5, 0);
        scene_grid->addWidget(head_radius_field_, 5, 1);
        builder_layout->addWidget(scene_group);

        auto* limits_group = new QGroupBox("Motion limits", scene_editor_);
        auto* limits_grid = new QGridLayout(limits_group);
        limits_grid->addWidget(new QLabel("X"), 0, 1);
        limits_grid->addWidget(new QLabel("Y"), 0, 2);
        limits_grid->addWidget(new QLabel("Z"), 0, 3);
        addVec3Row(limits_grid, 1, "max velocity", max_velocity_fields_);
        addVec3Row(limits_grid, 2, "max acceleration", max_acceleration_fields_);
        addVec3Row(limits_grid, 3, "max jerk", max_jerk_fields_);
        sample_dt_field_ = makeDoubleSpin(0.0001, 1.0, 0.004, 0.001, 4);
        limits_grid->addWidget(new QLabel("sample dt"), 4, 0);
        limits_grid->addWidget(sample_dt_field_, 4, 1);
        builder_layout->addWidget(limits_group);

        auto* obstacles_group = new QGroupBox("Obstacles", scene_editor_);
        auto* obstacles_layout = new QVBoxLayout(obstacles_group);
        obstacle_table_ = new QTableWidget(obstacles_group);
        obstacle_table_->setColumnCount(7);
        obstacle_table_->setHorizontalHeaderLabels({"type", "cx", "cy", "cz", "size/radius/file", "size_y/height", "size_z"});
        obstacle_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        obstacle_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        obstacle_table_->setSelectionMode(QAbstractItemView::SingleSelection);
        obstacles_layout->addWidget(obstacle_table_);

        auto* obstacle_buttons = new QGridLayout();
        add_box_obstacle_ = new QPushButton("Add box", obstacles_group);
        add_sphere_obstacle_ = new QPushButton("Add sphere", obstacles_group);
        add_cylinder_obstacle_ = new QPushButton("Add cylinder", obstacles_group);
        add_mesh_obstacle_ = new QPushButton("Add ascii STL", obstacles_group);
        remove_obstacle_ = new QPushButton("Remove selected", obstacles_group);
        save_scene_config_ = new QPushButton("Save scene config", obstacles_group);
        for (auto* button : {add_box_obstacle_, add_sphere_obstacle_, add_cylinder_obstacle_,
                             add_mesh_obstacle_, remove_obstacle_, save_scene_config_}) {
            button->setMinimumWidth(135);
            button->setMinimumHeight(28);
        }
        obstacle_buttons->addWidget(add_box_obstacle_, 0, 0);
        obstacle_buttons->addWidget(add_sphere_obstacle_, 0, 1);
        obstacle_buttons->addWidget(add_cylinder_obstacle_, 0, 2);
        obstacle_buttons->addWidget(add_mesh_obstacle_, 1, 0);
        obstacle_buttons->addWidget(remove_obstacle_, 1, 1);
        obstacle_buttons->addWidget(save_scene_config_, 1, 2);
        obstacle_buttons->setColumnStretch(0, 1);
        obstacle_buttons->setColumnStretch(1, 1);
        obstacle_buttons->setColumnStretch(2, 1);
        obstacles_layout->addLayout(obstacle_buttons);
        builder_layout->addWidget(obstacles_group, 1);

        builder_layout->addWidget(new QLabel("Edit the scene here first, then save it to update the JSON input and 3D view."));

        root->addWidget(controls);
        root->addWidget(tabs_);
        root->addWidget(scene_editor_);
        root->setStretchFactor(0, 0);
        root->setStretchFactor(1, 1);
        root->setStretchFactor(2, 0);

        statusBar()->showMessage("Ready");
    }

    void connectUi() {
        connect(load_preview_, &QPushButton::clicked, this, [this]() { loadConfigPreview(); });
        connect(run_plan_, &QPushButton::clicked, this, [this]() { runPlanning(); });
        connect(run_visualization_, &QPushButton::clicked, this, [this]() { runVisualization(); });
        connect(reset_view_, &QPushButton::clicked, this, [this]() { scene_view_->resetView(); });
        connect(add_box_obstacle_, &QPushButton::clicked, this, [this]() {
            addObstacleRow("box", 100.0, 100.0, 50.0, "60", "60", "100");
        });
        connect(add_sphere_obstacle_, &QPushButton::clicked, this, [this]() {
            addObstacleRow("sphere", 120.0, 120.0, 80.0, "20", "", "");
        });
        connect(add_cylinder_obstacle_, &QPushButton::clicked, this, [this]() {
            addObstacleRow("cylinder", 120.0, 120.0, 50.0, "15", "120", "");
        });
        connect(add_mesh_obstacle_, &QPushButton::clicked, this, [this]() { addMeshObstacle(); });
        connect(remove_obstacle_, &QPushButton::clicked, this, [this]() { removeSelectedObstacle(); });
        connect(save_scene_config_, &QPushButton::clicked, this, [this]() { saveSceneConfig(); });
    }

    void loadConfigPreview() {
        QFile file(config_path_->text());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            generated_config_json_ = {};
            config_dirty_ = false;
            appendLog("Cannot read config file: " + config_path_->text());
            return;
        }
        generated_config_json_ = QString::fromUtf8(file.readAll());
        config_dirty_ = false;

        try {
            last_problem_ = airmove::loadPlanningProblemJson(config_path_->text().toStdString());
            const auto& cfg = last_problem_.planner_config;
            planner_type_->setCurrentText(QString::fromStdString(cfg.planner_type));
            planning_time_->setValue(cfg.planning_time_limit);
            validity_resolution_->setValue(cfg.validity_resolution);
            smoothing_samples_->setValue(cfg.smoothing_samples);
            safety_margin_->setValue(cfg.safety_margin);
            simplify_solution_->setChecked(cfg.simplify_solution);
            populateBuilderFromProblem(last_problem_);
            scene_view_->setScene(last_problem_, last_result_);
            appendLog("Config loaded: " + config_path_->text());
        } catch (const std::exception& e) {
            appendLog(QString("Config parse failed: %1").arg(e.what()));
        }
    }

    void populateBuilderFromProblem(const airmove::PlanningProblem& problem) {
        writeVec3Fields(workspace_min_fields_, problem.planner_config.workspace_min);
        writeVec3Fields(workspace_max_fields_, problem.planner_config.workspace_max);
        writeVec3Fields(start_fields_, problem.request.start.position);
        writeVec3Fields(goal_fields_, problem.request.goal.position);
        writeVec3Fields(max_velocity_fields_, problem.request.limits.max_velocity);
        writeVec3Fields(max_acceleration_fields_, problem.request.limits.max_acceleration);
        writeVec3Fields(max_jerk_fields_, problem.request.limits.max_jerk);
        head_radius_field_->setValue(problem.planner_config.head_radius);
        sample_dt_field_->setValue(problem.request.sample_dt);

        obstacle_table_->setRowCount(0);
        for (const auto& box : problem.box_obstacles) {
            addObstacleRow("box", box.center.x(), box.center.y(), box.center.z(),
                           QString::number(box.size.x(), 'f', 3),
                           QString::number(box.size.y(), 'f', 3),
                           QString::number(box.size.z(), 'f', 3));
        }
        for (const auto& sphere : problem.sphere_obstacles) {
            addObstacleRow("sphere", sphere.center.x(), sphere.center.y(), sphere.center.z(),
                           QString::number(sphere.radius, 'f', 3), "", "");
        }
        for (const auto& cylinder : problem.cylinder_obstacles) {
            addObstacleRow("cylinder", cylinder.center.x(), cylinder.center.y(), cylinder.center.z(),
                           QString::number(cylinder.radius, 'f', 3),
                           QString::number(cylinder.height, 'f', 3), "");
        }
        for (const auto& mesh : problem.mesh_obstacles) {
            addObstacleRow("ascii_stl", 0.0, 0.0, 0.0, QString::fromStdString(mesh.file), "", "");
        }
    }

    void addObstacleRow(const QString& type,
                        double cx,
                        double cy,
                        double cz,
                        const QString& a,
                        const QString& b,
                        const QString& c) {
        const int row = obstacle_table_->rowCount();
        obstacle_table_->insertRow(row);
        setObstacleItem(row, kObstacleTypeCol, type);
        setObstacleItem(row, kObstacleXCol, QString::number(cx, 'f', 3));
        setObstacleItem(row, kObstacleYCol, QString::number(cy, 'f', 3));
        setObstacleItem(row, kObstacleZCol, QString::number(cz, 'f', 3));
        setObstacleItem(row, kObstacleACol, a);
        setObstacleItem(row, kObstacleBCol, b);
        setObstacleItem(row, kObstacleCCol, c);
    }

    void addMeshObstacle() {
        const QString path = QFileDialog::getOpenFileName(this, "Choose ascii STL", findProjectRoot(config_path_->text()), "STL (*.stl)");
        if (path.isEmpty()) {
            return;
        }

        QString file = path;
        const QString config_dir = QFileInfo(config_path_->text()).absolutePath();
        if (!config_dir.isEmpty()) {
            file = QDir(config_dir).relativeFilePath(path);
        }
        addObstacleRow("ascii_stl", 0.0, 0.0, 0.0, file, "", "");
    }

    void removeSelectedObstacle() {
        const int row = obstacle_table_->currentRow();
        if (row >= 0) {
            obstacle_table_->removeRow(row);
        }
    }

    void setObstacleItem(int row, int column, const QString& value) {
        auto* item = new QTableWidgetItem(value);
        if (column == kObstacleTypeCol) {
            item->setFlags(item->flags() | Qt::ItemIsEditable);
        }
        obstacle_table_->setItem(row, column, item);
    }

    QString obstacleText(int row, int column) const {
        const auto* item = obstacle_table_->item(row, column);
        return item == nullptr ? QString() : item->text().trimmed();
    }

    double obstacleDouble(int row, int column, double fallback = 0.0) const {
        bool ok = false;
        const double value = obstacleText(row, column).toDouble(&ok);
        return ok ? value : fallback;
    }

    static json vecToJson(const airmove::Vec3& value) {
        return json::array({value.x(), value.y(), value.z()});
    }

    bool applyEditorToConfig() {
        try {
            json root;
            root["workspace"] = {
                {"min", vecToJson(readVec3Fields(workspace_min_fields_))},
                {"max", vecToJson(readVec3Fields(workspace_max_fields_))},
            };
            root["tool"] = {{"head_radius", head_radius_field_->value()}};
            root["planning"] = {
                {"planner", planner_type_->currentText().toStdString()},
                {"range", 0.0},
                {"goal_bias", 0.05},
                {"safety_margin", safety_margin_->value()},
                {"planning_time", planning_time_->value()},
                {"validity_resolution", validity_resolution_->value()},
                {"smoothing_samples", smoothing_samples_->value()},
                {"simplify_solution", simplify_solution_->isChecked()},
            };
            root["motion_limits"] = {
                {"max_velocity", vecToJson(readVec3Fields(max_velocity_fields_))},
                {"max_acceleration", vecToJson(readVec3Fields(max_acceleration_fields_))},
                {"max_jerk", vecToJson(readVec3Fields(max_jerk_fields_))},
                {"sample_dt", sample_dt_field_->value()},
            };
            root["request"] = {
                {"start", {{"xyz", vecToJson(readVec3Fields(start_fields_))}}},
                {"goal", {{"xyz", vecToJson(readVec3Fields(goal_fields_))}}},
            };

            root["obstacles"] = json::array();
            for (int row = 0; row < obstacle_table_->rowCount(); ++row) {
                const QString type = obstacleText(row, kObstacleTypeCol).toLower();
                if (type == "box") {
                    root["obstacles"].push_back({
                        {"type", "box"},
                        {"center", json::array({obstacleDouble(row, kObstacleXCol),
                                                 obstacleDouble(row, kObstacleYCol),
                                                 obstacleDouble(row, kObstacleZCol)})},
                        {"size", json::array({obstacleDouble(row, kObstacleACol, 1.0),
                                               obstacleDouble(row, kObstacleBCol, 1.0),
                                               obstacleDouble(row, kObstacleCCol, 1.0)})},
                    });
                } else if (type == "sphere") {
                    root["obstacles"].push_back({
                        {"type", "sphere"},
                        {"center", json::array({obstacleDouble(row, kObstacleXCol),
                                                 obstacleDouble(row, kObstacleYCol),
                                                 obstacleDouble(row, kObstacleZCol)})},
                        {"radius", obstacleDouble(row, kObstacleACol, 1.0)},
                    });
                } else if (type == "cylinder") {
                    root["obstacles"].push_back({
                        {"type", "cylinder"},
                        {"center", json::array({obstacleDouble(row, kObstacleXCol),
                                                 obstacleDouble(row, kObstacleYCol),
                                                 obstacleDouble(row, kObstacleZCol)})},
                        {"radius", obstacleDouble(row, kObstacleACol, 1.0)},
                        {"height", obstacleDouble(row, kObstacleBCol, 1.0)},
                    });
                } else if (type == "ascii_stl") {
                    const QString file = obstacleText(row, kObstacleACol);
                    if (file.isEmpty()) {
                        throw std::runtime_error("ascii_stl obstacle requires a file path.");
                    }
                    root["obstacles"].push_back({
                        {"type", "ascii_stl"},
                        {"file", file.toStdString()},
                    });
                } else {
                    const QString message = "Unsupported obstacle type in row " + QString::number(row + 1);
                    throw std::runtime_error(message.toStdString());
                }
            }

            generated_config_json_ = QString::fromStdString(root.dump(2));
            config_dirty_ = true;
            appendLog("Scene editor applied to config JSON.");
            return true;
        } catch (const std::exception& e) {
            QMessageBox::warning(this, "Scene editor error", e.what());
            appendLog(QString("Scene editor error: %1").arg(e.what()));
            return false;
        }
    }

    bool saveSceneConfig() {
        if (!applyEditorToConfig()) {
            return false;
        }
        if (!saveConfig(false, false)) {
            return false;
        }
        scene_view_->setScene(last_problem_, last_result_);
        statusBar()->showMessage("Scene config saved and 3D view updated");
        appendLog("Scene config saved from Scene editor.");
        return true;
    }

    void runPlanning() {
        setBusy(true);
        try {
            if (!applyEditorToConfig()) {
                setBusy(false);
                return;
            }
            if (config_dirty_) {
                saveConfig(false, false);
            }
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
            last_planning_elapsed_ms_ = elapsed;

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
        if (!applyEditorToConfig()) {
            return;
        }
        if (config_dirty_) {
            saveConfig(false, false);
        }
        const QString script = findProjectFile("tools/visualize_path.py", config_path_->text());
        if (script.isEmpty()) {
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

    bool saveConfig(bool save_as, bool apply_builder = true) {
        QString path = config_path_->text();
        if (save_as || path.isEmpty()) {
            path = QFileDialog::getSaveFileName(this, "Save config", path, "JSON (*.json)");
            if (path.isEmpty()) {
                return false;
            }
            config_path_->setText(path);
        }

        if (apply_builder && !applyEditorToConfig()) {
            return false;
        }

        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            QMessageBox::warning(this, "Save failed", "Cannot write config file:\n" + path);
            return false;
        }
        file.write(generated_config_json_.toUtf8());
        file.close();
        config_dirty_ = false;
        appendLog("Config saved: " + path);

        try {
            last_problem_ = airmove::loadPlanningProblemJson(path.toStdString());
            populateBuilderFromProblem(last_problem_);
            scene_view_->setScene(last_problem_, last_result_);
        } catch (const std::exception& e) {
            appendLog(QString("Saved config parse failed: %1").arg(e.what()));
        }
        return true;
    }

    void updateSummary(const airmove::PlanningResult& result) {
        const double duration = result.trajectory.empty() ? 0.0 : result.trajectory.back().time;
        success_label_->setText(yesNo(result.success));
        raw_label_->setText(QString::number(result.raw_path_length, 'f', 3));
        shortcut_label_->setText(QString::number(result.shortcut_path_length, 'f', 3));
        smooth_label_->setText(QString::number(result.smoothed_path_length, 'f', 3));
        planning_elapsed_label_->setText(last_planning_elapsed_ms_ >= 0
                                             ? QString("%1 ms").arg(last_planning_elapsed_ms_)
                                             : QString("-"));
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
        add_box_obstacle_->setEnabled(!busy);
        add_sphere_obstacle_->setEnabled(!busy);
        add_cylinder_obstacle_->setEnabled(!busy);
        add_mesh_obstacle_->setEnabled(!busy);
        remove_obstacle_->setEnabled(!busy);
        save_scene_config_->setEnabled(!busy);
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
    QLabel* planning_elapsed_label_{nullptr};
    QLabel* duration_label_{nullptr};
    QLabel* clearance_label_{nullptr};
    QLabel* smoothing_label_{nullptr};
    QTabWidget* tabs_{nullptr};
    SceneViewWidget* scene_view_{nullptr};
    QTableWidget* path_table_{nullptr};
    QWidget* scene_editor_{nullptr};
    std::array<QDoubleSpinBox*, 3> workspace_min_fields_{};
    std::array<QDoubleSpinBox*, 3> workspace_max_fields_{};
    std::array<QDoubleSpinBox*, 3> start_fields_{};
    std::array<QDoubleSpinBox*, 3> goal_fields_{};
    std::array<QDoubleSpinBox*, 3> max_velocity_fields_{};
    std::array<QDoubleSpinBox*, 3> max_acceleration_fields_{};
    std::array<QDoubleSpinBox*, 3> max_jerk_fields_{};
    QDoubleSpinBox* head_radius_field_{nullptr};
    QDoubleSpinBox* sample_dt_field_{nullptr};
    QTableWidget* obstacle_table_{nullptr};
    QPushButton* add_box_obstacle_{nullptr};
    QPushButton* add_sphere_obstacle_{nullptr};
    QPushButton* add_cylinder_obstacle_{nullptr};
    QPushButton* add_mesh_obstacle_{nullptr};
    QPushButton* remove_obstacle_{nullptr};
    QPushButton* save_scene_config_{nullptr};
    QTextEdit* log_{nullptr};
    QString generated_config_json_;
    bool config_dirty_{false};
    airmove::PlanningProblem last_problem_;
    airmove::PlanningResult last_result_;
    qint64 last_planning_elapsed_ms_{-1};
};

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
