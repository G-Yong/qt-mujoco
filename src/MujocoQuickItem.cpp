#include "MujocoQuickItem.h"
#include "QtPlatformUIAdapter.h"

#include "simulate.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>

#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QQuickWindow>
#include <QMouseEvent>
#include <QHoverEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QGuiApplication>
#include <QScreen>
#include <QDebug>
#include <QMetaObject>
#include <QThread>
#include <QSGSimpleTextureNode>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace {
constexpr double  syncMisalign       = 0.1;
constexpr double  simRefreshFraction = 0.7;
constexpr int     kErrorLength       = 1024;
using Seconds = std::chrono::duration<double>;
} // namespace

// ===========================================================================
// MujocoFboRenderer：Qt Quick scenegraph 渲染线程
// ===========================================================================
namespace {
class MujocoFboRenderer : public QQuickFramebufferObject::Renderer {
public:
    explicit MujocoFboRenderer() = default;

    QOpenGLFramebufferObject* createFramebufferObject(const QSize& size) override {
        QOpenGLFramebufferObjectFormat fmt;
        fmt.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        fmt.setSamples(0);
        return new QOpenGLFramebufferObject(size, fmt);
    }

    void synchronize(QQuickFramebufferObject* qitem) override {
        m_item = static_cast<MujocoQuickItem*>(qitem);
    }

    void render() override {
        if (!m_item) return;
        unsigned int srcTex = m_item->currentSourceTexture();
        QSize srcSize       = m_item->currentSourceSize();
        QOpenGLFramebufferObject* dst = framebufferObject();
        if (!dst) return;

        auto* glctx = QOpenGLContext::currentContext();
        if (!glctx) return;
        QOpenGLExtraFunctions* gl = glctx->extraFunctions();

        // 即便还没有源纹理，也至少把 Quick FBO 清成不透明背景，
        // 避免出现未定义内容
        gl->glViewport(0, 0, dst->width(), dst->height());
        gl->glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
        gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!srcTex || srcSize.isEmpty()) return;

        // 用一个临时 read FBO 把共享纹理 attach 上来，
        // blit 到 Quick 提供的 draw FBO
        if (!m_readFbo) {
            gl->glGenFramebuffers(1, &m_readFbo);
        }
        gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, m_readFbo);
        gl->glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, srcTex, 0);
        gl->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dst->handle());
        gl->glBlitFramebuffer(0, 0, srcSize.width(), srcSize.height(),
                              0, 0, dst->width(), dst->height(),
                              GL_COLOR_BUFFER_BIT, GL_LINEAR);
        // 解绑，避免影响 Quick 后续状态
        gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        gl->glBindFramebuffer(GL_FRAMEBUFFER, dst->handle());
        m_item->window()->resetOpenGLState();
    }

    ~MujocoFboRenderer() override {
        if (m_readFbo && QOpenGLContext::currentContext()) {
            QOpenGLContext::currentContext()->extraFunctions()
                ->glDeleteFramebuffers(1, &m_readFbo);
        }
    }

private:
    MujocoQuickItem* m_item = nullptr;
    unsigned int     m_readFbo = 0;
};
} // namespace

// ===========================================================================
// MujocoQuickItem
// ===========================================================================
MujocoQuickItem::MujocoQuickItem(QQuickItem* parent)
    : QQuickFramebufferObject(parent) {
    setMirrorVertically(true); // mjr 是 OpenGL bottom-up，Quick 绘制时翻一下
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptHoverEvents(true);
    setFlag(ItemHasContents, true);
    setActiveFocusOnTab(true);
}

MujocoQuickItem::~MujocoQuickItem() { stop(); }

QQuickFramebufferObject::Renderer* MujocoQuickItem::createRenderer() const {
    return new MujocoFboRenderer();
}

void MujocoQuickItem::setXmlPath(const QString& path) {
    if (m_xmlPath == path) return;
    m_xmlPath = path;
    emit xmlPathChanged();
    if (m_running.load()) loadModel(path);
}

unsigned int MujocoQuickItem::currentSourceTexture() const {
    return m_adapterRaw ? m_adapterRaw->offscreenColorTexture() : 0u;
}
QSize MujocoQuickItem::currentSourceSize() const {
    if (!m_adapterRaw) return {};
    return {m_adapterRaw->offscreenWidth(), m_adapterRaw->offscreenHeight()};
}

// ----------------------------------------------------------------- 启停 ----
void MujocoQuickItem::start(const QString& filename) {
    if (m_running.exchange(true)) {
        if (!filename.isEmpty()) loadModel(filename);
        return;
    }

    if (!filename.isEmpty()) {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = filename;
        m_hasPendingLoad.store(true);
    } else if (!m_xmlPath.isEmpty()) {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = m_xmlPath;
        m_hasPendingLoad.store(true);
    }

    // 1) 离屏 surface（QWindow-less 渲染）
    m_surface = new QOffscreenSurface();
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    if (fmt.majorVersion() < 3) {
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    }
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    m_surface->setFormat(fmt);
    m_surface->create();

    // 2) GL context，必须 Qt 主线程创建；与 Qt Quick 共享 (Qt::AA_ShareOpenGLContexts)
    m_ctx = new QOpenGLContext();
    m_ctx->setFormat(fmt);
    m_ctx->setShareContext(QOpenGLContext::globalShareContext());
    if (!m_ctx->create()) {
        qWarning() << "MujocoQuickItem: GL context create failed";
        return;
    }

    // 推到渲染线程。这里两步走：先从当前（主）线程释放所有权
    // 到 nullptr（这是 moveToThread 对跨线程交接的唯一合法路径），
    // 渲染线程启动后再接手 moveToThread(currentThread()).
    m_ctx->moveToThread(nullptr);
    m_surface->moveToThread(nullptr);

    // 在 Quick 主线程把当前几何信息推给 adapter（adapter 还未存在时也无所谓）
    updateGeometryToAdapter();

    m_renderThread  = std::thread(&MujocoQuickItem::renderThreadMain,  this);
    m_physicsThread = std::thread(&MujocoQuickItem::physicsThreadMain, this);
}

void MujocoQuickItem::stop() {
    if (!m_running.exchange(false)) return;

    if (m_sim) m_sim->exitrequest.store(1);
    if (m_adapterRaw) m_adapterRaw->PostClose();

    if (m_physicsThread.joinable()) m_physicsThread.join();
    if (m_renderThread.joinable())  m_renderThread.join();

    m_sim.reset();
    // 渲染线程退出前已把 m_ctx / m_surface 的线程亲和性释放为 nullptr，
    // 这里可以安全地从当前（主）线程拾起、销毁。
    if (m_ctx) {
        m_ctx->moveToThread(QThread::currentThread());
        delete m_ctx;
        m_ctx = nullptr;
    }
    if (m_surface) {
        m_surface->moveToThread(QThread::currentThread());
        m_surface->destroy();
        delete m_surface;
        m_surface = nullptr;
    }
    m_adapterRaw = nullptr;
}

void MujocoQuickItem::loadModel(const QString& filename) {
    std::lock_guard<std::mutex> lk(m_pendingMtx);
    m_pendingFile = filename;
    m_hasPendingLoad.store(true);
}

// ----------------------------------------------------------------- IMujocoHost
void MujocoQuickItem::onFrameRendered() {
    // 在 mujoco 渲染线程被调用，转发到 GUI 线程触发 update()
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}
void MujocoQuickItem::onSetTitle(const QString& t) {
    QMetaObject::invokeMethod(this, [this, t] {
        if (window()) window()->setTitle(t);
    }, Qt::QueuedConnection);
}
void MujocoQuickItem::onToggleFullscreen() {
    QMetaObject::invokeMethod(this, [this] {
        QQuickWindow* w = window();
        if (!w) return;
        w->setVisibility(w->visibility() == QWindow::FullScreen
                         ? QWindow::Windowed : QWindow::FullScreen);
    }, Qt::QueuedConnection);
}

// ----------------------------------------------------------------- 几何 ----
void MujocoQuickItem::updateGeometryToAdapter() {
    if (!m_adapterRaw) return;
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    int w  = int(width()), h = int(height());
    int fbw = int(width() * dpr), fbh = int(height() * dpr);
    if (fbw <= 0) fbw = 1;
    if (fbh <= 0) fbh = 1;
    QScreen* scr = window() ? window()->screen() : QGuiApplication::primaryScreen();
    double dpi = scr ? scr->logicalDotsPerInch() : 96.0;
    m_adapterRaw->SetWindowGeometry(w, h, fbw, fbh, dpi);
    m_adapterRaw->PostResize(fbw, fbh);
}

#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
void MujocoQuickItem::geometryChange(const QRectF& newGeo, const QRectF& oldGeo) {
    QQuickFramebufferObject::geometryChange(newGeo, oldGeo);
    updateGeometryToAdapter();
}
#else
void MujocoQuickItem::geometryChanged(const QRectF& newGeo, const QRectF& oldGeo) {
    QQuickFramebufferObject::geometryChanged(newGeo, oldGeo);
    updateGeometryToAdapter();
}
#endif

void MujocoQuickItem::itemChange(ItemChange change, const ItemChangeData& data) {
    QQuickFramebufferObject::itemChange(change, data);
    if (change == ItemSceneChange) {
        updateGeometryToAdapter();
    }
}

// ----------------------------------------------------------------- 输入 ----
int MujocoQuickItem::qtMouseButtonToInternal(int btn) const {
    if (btn == Qt::LeftButton)   return 1;
    if (btn == Qt::RightButton)  return 2;
    if (btn == Qt::MiddleButton) return 3;
    return 0;
}
void MujocoQuickItem::updateModifiersFrom(int qtMods) {
    if (!m_adapterRaw) return;
    m_adapterRaw->SetModifiers(qtMods & Qt::ControlModifier,
                               qtMods & Qt::ShiftModifier,
                               qtMods & Qt::AltModifier);
}

void MujocoQuickItem::mousePressEvent(QMouseEvent* e) {
    forceActiveFocus();
    if (!m_adapterRaw) { QQuickFramebufferObject::mousePressEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 1,
                                  e->pos().x() * dpr, e->pos().y() * dpr);
    e->accept();
}
void MujocoQuickItem::mouseReleaseEvent(QMouseEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::mouseReleaseEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 0,
                                  e->pos().x() * dpr, e->pos().y() * dpr);
    e->accept();
}
void MujocoQuickItem::mouseMoveEvent(QMouseEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::mouseMoveEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    m_adapterRaw->PostMouseMove(e->pos().x() * dpr, e->pos().y() * dpr);
    e->accept();
}
void MujocoQuickItem::hoverMoveEvent(QHoverEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::hoverMoveEvent(e); return; }
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    m_adapterRaw->PostMouseMove(e->pos().x() * dpr, e->pos().y() * dpr);
    e->accept();
}
void MujocoQuickItem::wheelEvent(QWheelEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::wheelEvent(e); return; }
    QPoint d = e->angleDelta();
    m_adapterRaw->PostScroll(d.x() / 120.0, d.y() / 120.0);
    e->accept();
}
int MujocoQuickItem::qtKeyToMjui(int key) const {
    if (key >= Qt::Key_Space && key <= Qt::Key_AsciiTilde) return key;
    switch (key) {
        case Qt::Key_Escape:    return 256;
        case Qt::Key_Enter:
        case Qt::Key_Return:    return 257;
        case Qt::Key_Tab:       return 258;
        case Qt::Key_Backspace: return 259;
        case Qt::Key_Insert:    return 260;
        case Qt::Key_Delete:    return 261;
        case Qt::Key_Right:     return 262;
        case Qt::Key_Left:      return 263;
        case Qt::Key_Down:      return 264;
        case Qt::Key_Up:        return 265;
        case Qt::Key_PageUp:    return 266;
        case Qt::Key_PageDown:  return 267;
        case Qt::Key_Home:      return 268;
        case Qt::Key_End:       return 269;
        case Qt::Key_F1:        return 290;
        case Qt::Key_F2:        return 291;
        case Qt::Key_F3:        return 292;
        case Qt::Key_F4:        return 293;
        case Qt::Key_F5:        return 294;
        case Qt::Key_F6:        return 295;
        case Qt::Key_F7:        return 296;
        case Qt::Key_F8:        return 297;
        case Qt::Key_F9:        return 298;
        case Qt::Key_F10:       return 299;
        case Qt::Key_F11:       return 300;
        case Qt::Key_F12:       return 301;
        default:                return 0;
    }
}
void MujocoQuickItem::keyPressEvent(QKeyEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::keyPressEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 1);
    e->accept();
}
void MujocoQuickItem::keyReleaseEvent(QKeyEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::keyReleaseEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 0);
    e->accept();
}

// ----------------------------------------------------------------- 渲染线程 ----
void MujocoQuickItem::renderThreadMain() {
    m_ctx->moveToThread(QThread::currentThread());
    m_surface->moveToThread(QThread::currentThread());
    if (!m_ctx->makeCurrent(m_surface)) {
        qWarning() << "MujocoQuickItem: makeCurrent failed";
        return;
    }

    static mjvCamera  cam;  mjv_defaultCamera(&cam);
    static mjvOption  opt;  mjv_defaultOption(&opt);
    static mjvPerturb pert; mjv_defaultPerturb(&pert);

    auto adapter_unique = std::make_unique<mjqt::QtPlatformUIAdapter>(this);
    m_adapterRaw = adapter_unique.get();

    m_sim = std::make_unique<mujoco::Simulate>(
        std::move(adapter_unique),
        &cam, &opt, &pert, /*is_passive=*/false);

    m_sim->vsync = 0; // 离屏渲染，不需要垂直同步

    // adapter 已就绪，把当前几何信息再推一次
    QMetaObject::invokeMethod(this, [this] { updateGeometryToAdapter(); },
                              Qt::QueuedConnection);

    m_sim->RenderLoop();

    // 释放适配器持有的 GL 资源（共享纹理 / FBO），此时 GL context 仍在当前线程
    if (m_adapterRaw) m_adapterRaw->ReleaseSharedGL();

    m_ctx->doneCurrent();

    // 把 QObject 亲和性交还为 nullptr，主线程会在 stop() 中拾回并销毁。
    // 这是 Qt 跨线程交接 QObject 的合法做法；moveToThread 必须
    // 在当前拥有该对象的线程中调用。
    m_ctx->moveToThread(nullptr);
    m_surface->moveToThread(nullptr);
}

// ----------------------------------------------------------------- 物理线程 ----
namespace {
mjModel* loadModelFile(const QString& filename, mujoco::Simulate& sim) {
    char err[kErrorLength] = "";
    QByteArray utf8 = filename.toLocal8Bit();
    mjModel* m = nullptr;
    if (filename.endsWith(".mjb", Qt::CaseInsensitive)) {
        m = mj_loadModel(utf8.constData(), nullptr);
        if (!m) std::strncpy(err, "could not load binary model", sizeof(err) - 1);
    } else if (filename.endsWith(".xml", Qt::CaseInsensitive)) {
        m = mj_loadXML(utf8.constData(), nullptr, err, sizeof(err));
    } else {
        mjSpec* spec = mj_parse(utf8.constData(), nullptr, nullptr, err, sizeof(err));
        if (spec) { m = mj_compile(spec, nullptr); mj_deleteSpec(spec); }
    }
    if (!m) {
        std::strncpy(sim.load_error, err, sizeof(sim.load_error) - 1);
        std::printf("loadModel error: %s\n", err);
    }
    return m;
}
} // namespace

void MujocoQuickItem::physicsThreadMain() {
    while (m_running.load() && !m_sim) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!m_sim) return;

    auto& sim = *m_sim;
    mjModel* m = nullptr;
    mjData*  d = nullptr;

    using Clock = mujoco::Simulate::Clock;
    std::chrono::time_point<Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim.exitrequest.load() && m_running.load()) {
        if (m_hasPendingLoad.exchange(false)) {
            QString file;
            { std::lock_guard<std::mutex> lk(m_pendingMtx); file = m_pendingFile; }
            sim.LoadMessage(file.toLocal8Bit().constData());
            mjModel* mnew = loadModelFile(file, sim);
            mjData*  dnew = mnew ? mj_makeData(mnew) : nullptr;
            if (dnew) {
                sim.Load(mnew, dnew, file.toLocal8Bit().constData());
                std::unique_lock<std::recursive_mutex> lk(sim.mtx);
                if (d) mj_deleteData(d);
                if (m) mj_deleteModel(m);
                m = mnew; d = dnew;
                mj_forward(m, d);
            } else {
                sim.LoadMessageClear();
            }
        }

        if (sim.uiloadrequest.load()) {
            sim.uiloadrequest.fetch_sub(1);
            sim.LoadMessage(sim.filename);
            mjModel* mnew = loadModelFile(QString::fromLocal8Bit(sim.filename), sim);
            mjData*  dnew = mnew ? mj_makeData(mnew) : nullptr;
            if (dnew) {
                sim.Load(mnew, dnew, sim.filename);
                std::unique_lock<std::recursive_mutex> lk(sim.mtx);
                if (d) mj_deleteData(d);
                if (m) mj_deleteModel(m);
                m = mnew; d = dnew;
                mj_forward(m, d);
            } else {
                sim.LoadMessageClear();
            }
        }

        if (sim.run && sim.busywait) std::this_thread::yield();
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));

        std::unique_lock<std::recursive_mutex> lk(sim.mtx);
        if (!m) continue;

        if (sim.run) {
            const auto startCPU   = Clock::now();
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim     = d->time - syncSim;
            double slowdown       = 100.0 / sim.percentRealTime[sim.real_time_index];
            bool misaligned = std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

            if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
                syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed) {
                syncCPU = startCPU; syncSim = d->time;
                sim.speed_changed = false;
                sim.InjectNoise(sim.key);
                mj_step(m, d);
                sim.AddToHistory();
            } else {
                mjtNum prevSim = d->time;
                double refreshTime = simRefreshFraction / sim.refresh_rate;
                while (Seconds((d->time - syncSim)*slowdown) < Clock::now() - syncCPU &&
                       Clock::now() - startCPU < Seconds(refreshTime)) {
                    sim.InjectNoise(sim.key);
                    mj_step(m, d);
                    if (d->time < prevSim) break;
                }
                sim.AddToHistory();
            }
        } else {
            mj_forward(m, d);
            if (sim.pause_update) mju_copy(d->qacc_warmstart, d->qacc, m->nv);
            sim.speed_changed = true;
        }
    }

    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
}
