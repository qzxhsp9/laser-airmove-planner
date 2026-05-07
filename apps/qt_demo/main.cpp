#include "airmove/AirMovePlanner.hpp"
#include "airmove/JsonIO.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDir>
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
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTime>
#include <QVBoxLayout>

#include <stdexcept>

namespace {

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

class MainWindow : public QMainWindow {
public:
    MainWindow() {
        setWindowTitle("Laser Air-Move Planner Qt Demo");
        resize(1280, 820);
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

        auto* io_group = new QGroupBox("输入输出", controls);
        auto* io_layout = new QFormLayout(io_group);
        config_path_ = new QLineEdit(io_group);
        output_path_ = new QLineEdit(io_group);
        auto* browse_config = new QPushButton("选择配置", io_group);
        auto* browse_output = new QPushButton("选择输出目录", io_group);
        io_layout->addRow("配置文件", config_path_);
        io_layout->addRow("", browse_config);
        io_layout->addRow("输出目录", output_path_);
        io_layout->addRow("", browse_output);
        controls_layout->addWidget(io_group);

        auto* params_group = new QGroupBox("轻量调参", controls);
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
        simplify_solution_ = new QCheckBox("启用 OMPL simplifySolution", params_group);

        params_layout->addRow("planner", planner_type_);
        params_layout->addRow("planning_time", planning_time_);
        params_layout->addRow("validity_resolution", validity_resolution_);
        params_layout->addRow("smoothing_samples", smoothing_samples_);
        params_layout->addRow("safety_margin", safety_margin_);
        params_layout->addRow("", simplify_solution_);
        controls_layout->addWidget(params_group);

        auto* actions_group = new QGroupBox("操作", controls);
        auto* actions_layout = new QVBoxLayout(actions_group);
        run_plan_ = new QPushButton("运行规划", actions_group);
        run_visualization_ = new QPushButton("生成可视化", actions_group);
        load_preview_ = new QPushButton("重新读取配置", actions_group);
        actions_layout->addWidget(run_plan_);
        actions_layout->addWidget(run_visualization_);
        actions_layout->addWidget(load_preview_);
        controls_layout->addWidget(actions_group);

        auto* metrics_group = new QGroupBox("规划摘要", controls);
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
                this, "选择规划配置", QDir::currentPath(), "JSON (*.json)");
            if (!path.isEmpty()) {
                config_path_->setText(path);
                loadConfigPreview();
            }
        });
        connect(browse_output, &QPushButton::clicked, this, [this]() {
            const QString path = QFileDialog::getExistingDirectory(this, "选择输出目录", output_path_->text());
            if (!path.isEmpty()) {
                output_path_->setText(path);
            }
        });

        tabs_ = new QTabWidget(root);
        image_label_ = new QLabel(tabs_);
        image_label_->setAlignment(Qt::AlignCenter);
        image_label_->setMinimumSize(600, 420);
        auto* scroll = new QScrollArea(tabs_);
        scroll->setWidget(image_label_);
        scroll->setWidgetResizable(true);
        tabs_->addTab(scroll, "路径预览");

        path_table_ = new QTableWidget(tabs_);
        path_table_->setColumnCount(4);
        path_table_->setHorizontalHeaderLabels({"index", "x", "y", "z"});
        path_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tabs_->addTab(path_table_, "smoothed path");

        config_preview_ = new QTextEdit(tabs_);
        config_preview_->setReadOnly(true);
        tabs_->addTab(config_preview_, "配置预览");

        log_ = new QTextEdit(tabs_);
        log_->setReadOnly(true);
        tabs_->addTab(log_, "日志");

        root->addWidget(controls);
        root->addWidget(tabs_);
        root->setStretchFactor(1, 1);

        statusBar()->showMessage("Ready");
    }

    void connectUi() {
        connect(load_preview_, &QPushButton::clicked, this, [this]() { loadConfigPreview(); });
        connect(run_plan_, &QPushButton::clicked, this, [this]() { runPlanning(); });
        connect(run_visualization_, &QPushButton::clicked, this, [this]() { runVisualization(); });
    }

    void loadConfigPreview() {
        QFile file(config_path_->text());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            config_preview_->setPlainText("无法读取配置文件。");
            return;
        }
        config_preview_->setPlainText(QString::fromUtf8(file.readAll()));

        try {
            const auto problem = airmove::loadPlanningProblemJson(config_path_->text().toStdString());
            const auto& cfg = problem.planner_config;
            planner_type_->setCurrentText(QString::fromStdString(cfg.planner_type));
            planning_time_->setValue(cfg.planning_time_limit);
            validity_resolution_->setValue(cfg.validity_resolution);
            smoothing_samples_->setValue(cfg.smoothing_samples);
            safety_margin_->setValue(cfg.safety_margin);
            simplify_solution_->setChecked(cfg.simplify_solution);
            appendLog("配置已读取: " + config_path_->text());
        } catch (const std::exception& e) {
            appendLog(QString("配置解析失败: %1").arg(e.what()));
        }
    }

    void runPlanning() {
        setBusy(true);
        try {
            auto problem = airmove::loadPlanningProblemJson(config_path_->text().toStdString());
            problem.planner_config.planner_type = planner_type_->currentText().toStdString();
            problem.planner_config.planning_time_limit = planning_time_->value();
            problem.request.planning_time = planning_time_->value();
            problem.planner_config.validity_resolution = validity_resolution_->value();
            problem.planner_config.smoothing_samples = smoothing_samples_->value();
            problem.planner_config.safety_margin = safety_margin_->value();
            problem.request.safety_margin = safety_margin_->value();
            problem.planner_config.simplify_solution = simplify_solution_->isChecked();

            auto world = airmove::buildCollisionWorld(problem);
            airmove::AirMovePlanner planner(problem.planner_config);
            const auto begin = QDateTime::currentMSecsSinceEpoch();
            last_result_ = planner.plan(problem.request, world);
            const auto elapsed = QDateTime::currentMSecsSinceEpoch() - begin;

            airmove::writePlanningOutputs(output_path_->text().toStdString(), last_result_);
            updateSummary(last_result_);
            updatePathTable(last_result_.smoothed_path);
            appendLog(QString("规划完成，耗时 %1 ms: %2")
                          .arg(elapsed)
                          .arg(QString::fromStdString(last_result_.message)));
            statusBar()->showMessage(last_result_.success ? "Planning succeeded" : "Planning failed");
            tabs_->setCurrentIndex(1);
        } catch (const std::exception& e) {
            appendLog(QString("规划异常: %1").arg(e.what()));
            QMessageBox::critical(this, "规划异常", e.what());
        }
        setBusy(false);
    }

    void runVisualization() {
        const QString script = QDir::current().absoluteFilePath("tools/visualize_path.py");
        if (!QFileInfo::exists(script)) {
            QMessageBox::warning(this, "缺少脚本", "未找到 tools/visualize_path.py");
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

        appendLog("开始生成可视化图片...");
        QProcess process;
        process.setProgram("py");
        process.setArguments(args);
        process.setProcessChannelMode(QProcess::MergedChannels);
        process.start();
        if (!process.waitForStarted(3000)) {
            QMessageBox::warning(this, "启动失败", "无法启动 Python，可手动运行 tools/visualize_path.py。");
            return;
        }
        process.waitForFinished(120000);
        appendLog(QString::fromLocal8Bit(process.readAll()));

        const QString image_path = QDir(image_dir).absoluteFilePath("path_3d.png");
        QPixmap pixmap(image_path);
        if (pixmap.isNull()) {
            QMessageBox::warning(this, "图片生成失败", "未生成 path_3d.png，请查看日志。");
            return;
        }
        image_label_->setPixmap(pixmap.scaled(
            image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        tabs_->setCurrentIndex(0);
        statusBar()->showMessage("Visualization generated");
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
    QPushButton* load_preview_{nullptr};
    QLabel* success_label_{nullptr};
    QLabel* raw_label_{nullptr};
    QLabel* shortcut_label_{nullptr};
    QLabel* smooth_label_{nullptr};
    QLabel* duration_label_{nullptr};
    QLabel* clearance_label_{nullptr};
    QLabel* smoothing_label_{nullptr};
    QTabWidget* tabs_{nullptr};
    QLabel* image_label_{nullptr};
    QTableWidget* path_table_{nullptr};
    QTextEdit* config_preview_{nullptr};
    QTextEdit* log_{nullptr};
    airmove::PlanningResult last_result_;
};

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
