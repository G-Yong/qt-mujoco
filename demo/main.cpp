#include "MujocoQuickItem.h"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QSurfaceFormat>

#if defined(_WIN32)
#  include <windows.h>
#endif

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

    QGuiApplication app(argc, argv);

    // 注册 QML 类型
    qmlRegisterType<MujocoQuickItem>("Mujoco", 1, 0, "MujocoView");

    QQmlApplicationEngine engine;
    // 默认模型路径 —— 改成你自己的路径
    engine.rootContext()->setContextProperty(
        "initialXmlPath",
        QStringLiteral("C:/Users/Administrator/Desktop/robotSim/qt-mujoco/"
                       "mujoco-3.8.0-windows-x86_64/model/cards/cards.xml"));

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty()) return -1;
    return app.exec();
}
