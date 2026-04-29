#pragma once
// ---------------------------------------------------------------------------
// SimulateView
//
// 一个普通的 QWidget，把官方 mujoco::Simulate 嵌入到 Qt 部件树。
// 使用 createWindowContainer 包装 QtSimulateWindow。
// ---------------------------------------------------------------------------

#include <QWidget>
#include <QString>

class QtSimulateWindow;

class SimulateView : public QWidget {
    Q_OBJECT
public:
    explicit SimulateView(QWidget *parent = nullptr);
    ~SimulateView() override;

    // 启动 Simulate（创建渲染/物理线程）
    void start(const QString &xmlPath = QString());

    // 异步请求加载新模型；线程安全
    void loadModel(const QString &xmlPath);

protected:
    void dragEnterEvent(QDragEnterEvent *e) override;
    void dropEvent(QDropEvent *e) override;
    bool eventFilter(QObject *obj, QEvent *e) override;

private:
    QtSimulateWindow *m_window = nullptr;
};
