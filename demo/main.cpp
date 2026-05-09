#include "MujocoQuickItem.h"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QSurfaceFormat>
#include <QQuickWidget>
#include <QApplication>
#include <QLabel>
#include <QTimer>

#include <mujoco/mujoco.h>

#include <algorithm>

namespace {
QString mujocoObjectName(const mjModel* model, int objectType, int id)
{
    if (!model || id < 0) return QStringLiteral("-");

    const char* name = mj_id2name(model, objectType, id);
    if (name && name[0] != '\0') return QString::fromUtf8(name);

    return QStringLiteral("#%1").arg(id);
}

QString geomDescription(const mjModel* model, int geomId)
{
    if (!model || geomId < 0) return QStringLiteral("flex / none");

    const int bodyId = model->geom_bodyid[geomId];
    return QStringLiteral("%1 (body %2)")
        .arg(mujocoObjectName(model, mjOBJ_GEOM, geomId),
             mujocoObjectName(model, mjOBJ_BODY, bodyId));
}

QString vec3Text(const mjtNum* value)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(static_cast<double>(value[0]), 0, 'f', 3)
        .arg(static_cast<double>(value[1]), 0, 'f', 3)
        .arg(static_cast<double>(value[2]), 0, 'f', 3);
}

QString collisionSummary(MujocoQuickItem* mujoco)
{
    if (!mujoco) return QStringLiteral("未找到 MujocoView");

    QString summary = QStringLiteral("模型尚未加载");

    // withSimulation 会在 sim.mtx 锁内执行回调。这里仅做短时间只读快照，
    // 读取 d->ncon 和 d->contact[i] 即可判断当前 step 是否检测到接触。
    mujoco->withSimulation([&summary](const mjModel* model, mjData* data) {
        if (data->ncon <= 0) {
            summary = QStringLiteral("碰撞: 无\ncontacts: 0");
            return;
        }

        int activeContacts = 0; // 只有 exclude=0 且 efc_address>=0 的 contact 才会产生约束反作用力，算作 active contact
        int penetratingContacts = 0; // dist<0 的 contact 是穿透状态，算作 penetrating contact
        for (int contactIndex = 0; contactIndex < data->ncon; ++contactIndex) {
            const mjContact& contact = data->contact[contactIndex];
            if (contact.exclude == 0 && contact.efc_address >= 0) ++activeContacts;
            if (contact.dist < 0) ++penetratingContacts;
        }

        summary = QStringLiteral("碰撞/接触: 有\ncontacts: %1  active: %2  penetrating: %3")
            .arg(data->ncon)
            .arg(activeContacts)
            .arg(penetratingContacts);

        const int shownContacts = std::min(data->ncon, 4);
        for (int contactIndex = 0; contactIndex < shownContacts; ++contactIndex) {
            const mjContact& contact = data->contact[contactIndex];

            mjtNum force[6] = {0, 0, 0, 0, 0, 0};
            mj_contactForce(model, data, contactIndex, force);

            summary += QStringLiteral(
                "\n#%1 %2 <-> %3"
                "\n  dist=%4  normalForce=%5  active=%6"
                "\n  pos=%7")
                .arg(contactIndex)
                .arg(geomDescription(model, contact.geom[0]),
                     geomDescription(model, contact.geom[1]))
                .arg(static_cast<double>(contact.dist), 0, 'f', 6)
                .arg(static_cast<double>(force[0]), 0, 'f', 3)
                .arg(contact.exclude == 0 && contact.efc_address >= 0 ? QStringLiteral("yes") : QStringLiteral("no"))
                .arg(vec3Text(contact.pos));
        }

        if (data->ncon > shownContacts) {
            summary += QStringLiteral("\n... 还有 %1 个 contact").arg(data->ncon - shownContacts);
        }
    });

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

    QTimer *collisionTimer = new QTimer(view);
    QObject::connect(collisionTimer, &QTimer::timeout, view, [label, mujoco]() {
        label->setText(collisionSummary(mujoco));
        label->adjustSize();
    });
    collisionTimer->start(200);

    return app.exec();
}
