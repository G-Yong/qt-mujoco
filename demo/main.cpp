#include "MujocoQuickItem.h"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QSurfaceFormat>
#include <QQuickWidget>
#include <QApplication>
#include <QLabel>

namespace {
QString vec3Text(const QVector3D& v)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(static_cast<double>(v.x()), 0, 'f', 3)
        .arg(static_cast<double>(v.y()), 0, 'f', 3)
        .arg(static_cast<double>(v.z()), 0, 'f', 3);
}

QString collisionSummary(MujocoQuickItem* mujoco)
{
    if (!mujoco) return QStringLiteral("未找到 MujocoView");

    const int total = mujoco->contactCount();
    if (total <= 0)
        return QStringLiteral("碰撞: 无\ncontacts: 0");

    int activeCount = 0, penetratingCount = 0;
    for (int i = 0; i < total; ++i) {
        const ContactInfo c = mujoco->contact(i);
        if (c.active)      ++activeCount;
        if (c.penetrating) ++penetratingCount;
    }

    QString summary = QStringLiteral("碰撞/接触: 有\ncontacts: %1  active: %2  penetrating: %3")
        .arg(total).arg(activeCount).arg(penetratingCount);

    const int shown = std::min(total, 4);
    for (int i = 0; i < shown; ++i) {
        const ContactInfo c = mujoco->contact(i);
        summary += QStringLiteral(
            "\n#%1 %2 (body %3) <-> %4 (body %5)"
            "\n  dist=%6  normalForce=%7  active=%8"
            "\n  pos=%9")
            .arg(i)
            .arg(c.geom0Name, c.body0Name, c.geom1Name, c.body1Name)
            .arg(c.dist, 0, 'f', 6)
            .arg(c.normalForce, 0, 'f', 3)
            .arg(c.active ? QStringLiteral("yes") : QStringLiteral("no"))
            .arg(vec3Text(c.position));
    }
    if (total > shown)
        summary += QStringLiteral("\n... 还有 %1 个 contact").arg(total - shown);

    return summary;
}
} // namespace

#ifndef ASSETS_DIR
#define ASSETS_DIR "."
#endif

// ---------------------------------------------------------------------------
// 强制混合显卡（NVIDIA Optimus / AMD PowerXpress）选用独立 GPU
//
// 背景：在带集显 + 独显的 Windows 机器上，操作系统默认让进程跑在集显上。
//      若集显驱动缺失/异常，Windows 会回退到 "Microsoft GDI Generic" 软件
//      OpenGL 1.1（无 ARB_framebuffer_object），导致 MuJoCo 的
//      mjr_makeContext 报 "ERROR: OpenGL ARB_framebuffer_object required"。
//
// 修复：在主可执行文件中导出下面两个符号，NVIDIA / AMD 驱动会识别并
//      自动把本进程切到独显（业界标准做法）。
// ---------------------------------------------------------------------------
#if defined(_WIN32)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

int main(int argc, char *argv[])
{
    // 使用系统硬件 OpenGL（独立 GPU 由文件头部的导出符号强制选定）
    QGuiApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    // 关键：让 Mujoco 渲染线程的私有 GL context 与 Qt Quick 的 scenegraph
    //       context 共享，从而可以跨线程使用同一个 GL 纹理。
    QGuiApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    // MuJoCo 渲染要求 OpenGL 3.3 Core 或更高的兼容上下文
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile); // mjr_* 用了部分固定管线兼容调用
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    // 注册 QML 类型
    qmlRegisterType<MujocoQuickItem>("Mujoco", 1, 0, "MujocoView");

    QQuickWidget *view = new QQuickWidget();
    view->setWindowTitle("MuJoCo in Qt Quick Demo");
    view->setResizeMode(QQuickWidget::SizeRootObjectToView);
    // 默认模型路径 —— 改成你自己的路径
    view->engine()->rootContext()->setContextProperty(
        "initialXmlPath",
        QStringLiteral("../../../../"
                       "mujoco-3.8.0-windows-x86_64/model/cards/cards.xml"));
    view->setSource(QUrl("qrc:/main.qml"));
    view->show();

    auto* mujoco = view->rootObject()
        ? view->rootObject()->findChild<MujocoQuickItem*>(QStringLiteral("mujocoView"))
        : nullptr;

    // 叠加一个纯文本标签，演示如何从 C++ 侧读取当前碰撞信息。
    QLabel *label = new QLabel(view);
    label->setStyleSheet("QLabel { color: white; background-color: rgba(0, 0, 0, 160); font-size: 13px; padding: 6px; border-radius: 4px; }");
    label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    label->setAttribute(Qt::WA_TransparentForMouseEvents); // 鼠标事件穿透
    label->setWordWrap(true);
    label->setMaximumWidth(620);
    label->setText(collisionSummary(mujoco));
    label->adjustSize();
    label->move(10, 10);
    label->show();

    // contactsChanged 每帧发出，直接驱动标签刷新，无需轮询定时器。
    if (mujoco) {
        QObject::connect(mujoco, &MujocoQuickItem::contactsChanged, view, [label, mujoco]() {
            label->setText(collisionSummary(mujoco));
            label->adjustSize();
        });
    }

    return app.exec();
}

