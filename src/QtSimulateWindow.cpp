#include "QtSimulateWindow.h"
#include "QtPlatformUIAdapter.h"

#include "simulate.h"               // 官方
// #include "platform_ui_adapter.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>

#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QExposeEvent>
#include <QEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QFileInfo>
#include <QGuiApplication>
#include <QScreen>
#include <QDebug>
#include <QMetaObject>
#include <QThread>

#include <chrono>
#include <cstdio>
#include <cstring>

// 与 main.cc 一致的常量
namespace {
constexpr double  syncMisalign      = 0.1;
constexpr double  simRefreshFraction= 0.7;
constexpr int     kErrorLength      = 1024;
using Seconds = std::chrono::duration<double>;
}

QtSimulateWindow::QtSimulateWindow(QWindow *parent) : QWindow(parent) {
    setSurfaceType(QSurface::OpenGLSurface);

    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    if (fmt.majorVersion() < 3) {
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    }
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4);
    setFormat(fmt);
}

QtSimulateWindow::~QtSimulateWindow() { stop(); }

// ----------------------------------------------------------------- 启停 ----
void QtSimulateWindow::start(const QString &filename) {
    if (m_running.exchange(true)) return;

    if (!filename.isEmpty()) {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = filename;
        m_hasPendingLoad.store(true);
    }

    // GL 上下文必须在 Qt 主线程创建，再 moveToThread 给渲染线程
    m_ctx = new QOpenGLContext();
    m_ctx->setFormat(format());
    m_ctx->create();

    create();   // 让 QWindow 拥有原生窗口，否则 makeCurrent 会失败
    updateGeometryToAdapter();   // 第一次同步几何信息（adapter 在这之前还没建）

    // 让 GL 上下文 / window 都不归属当前线程
    m_ctx->moveToThread(nullptr);   // 渲染线程会再 makeCurrent

    m_renderThread = std::thread(&QtSimulateWindow::renderThreadMain, this);
    m_physicsThread = std::thread(&QtSimulateWindow::physicsThreadMain, this);
}

void QtSimulateWindow::stop() {
    if (!m_running.exchange(false)) return;

    if (m_sim) m_sim->exitrequest.store(1);
    if (m_adapterRaw) m_adapterRaw->PostClose();

    if (m_physicsThread.joinable()) m_physicsThread.join();
    if (m_renderThread.joinable())  m_renderThread.join();

    m_sim.reset();
    delete m_ctx;
    m_ctx = nullptr;
}

void QtSimulateWindow::loadModel(const QString &filename) {
    std::lock_guard<std::mutex> lk(m_pendingMtx);
    m_pendingFile = filename;
    m_hasPendingLoad.store(true);
}

// ----------------------------------------------------------------- GL ----
void QtSimulateWindow::swapBuffersFromRenderThread() {
    if (!m_ctx) return;
    // 防御式：MuJoCo 的 mjr_* 内部会通过 GLAD 直接调 wglMakeCurrent，
    // 在某些驱动 / 多 surface 场景下会让 Qt 的 thread-local current 跟踪与
    // 原生当前上下文脱节，从而触发：
    //   "QOpenGLContext::swapBuffers() called without corresponding makeCurrent()"
    // 在 swap 前显式 makeCurrent 一次即可保持两者一致；若已是当前则是 no-op。
    if (QOpenGLContext::currentContext() != m_ctx) {
        m_ctx->makeCurrent(this);
    }
    m_ctx->swapBuffers(this);
}
void QtSimulateWindow::setVSyncFromRenderThread(bool on) {
    // wglSwapIntervalEXT 支持在 context current 时运行时切换，此处直接调用。
    // getProcAddress 返回当前 context 的扩展函数指针，无需额外 include Windows 头文件。
    auto* ctx = QOpenGLContext::currentContext();
    if (!ctx && m_ctx) {
        // 同 swapBuffers：保持 Qt 的 current-context 跟踪与原生状态一致
        m_ctx->makeCurrent(this);
        ctx = QOpenGLContext::currentContext();
    }
    if (!ctx) return;
    typedef int (*SwapIntervalFunc)(int);
    auto swapIntervalFn = reinterpret_cast<SwapIntervalFunc>(
        ctx->getProcAddress("wglSwapIntervalEXT"));
    if (swapIntervalFn) {
        swapIntervalFn(on ? 1 : 0);
    }
}

void QtSimulateWindow::postSetTitle(const QString &t) {
    QMetaObject::invokeMethod(this, [this, t] { setTitle(t); }, Qt::QueuedConnection);
}
void QtSimulateWindow::postToggleFullscreen() {
    QMetaObject::invokeMethod(this, [this] {
        setVisibility(visibility() == QWindow::FullScreen
                      ? QWindow::Windowed : QWindow::FullScreen);
    }, Qt::QueuedConnection);
}

// ----------------------------------------------------------------- Qt 事件 ----
bool QtSimulateWindow::event(QEvent *e) {
    if (e->type() == QEvent::Close && m_adapterRaw) m_adapterRaw->PostClose();

    if (e->type() == QEvent::DragEnter) {
        auto *de = static_cast<QDragEnterEvent *>(e);
        if (de->mimeData()->hasUrls()) {
            for (const QUrl &u : de->mimeData()->urls()) {
                if (u.isLocalFile()) {
                    const QString suf = QFileInfo(u.toLocalFile()).suffix().toLower();
                    if (suf == "xml" || suf == "mjb") {
                        de->acceptProposedAction();
                        return true;
                    }
                }
            }
        }
        return true; // consume even if not accepted
    }

    if (e->type() == QEvent::Drop) {
        auto *de = static_cast<QDropEvent *>(e);
        if (de->mimeData()->hasUrls()) {
            for (const QUrl &u : de->mimeData()->urls()) {
                if (u.isLocalFile()) {
                    const QString path = u.toLocalFile();
                    const QString suf  = QFileInfo(path).suffix().toLower();
                    if (suf == "xml" || suf == "mjb") {
                        loadModel(path);
                        de->acceptProposedAction();
                        return true;
                    }
                }
            }
        }
        return true;
    }

    return QWindow::event(e);
}
void QtSimulateWindow::exposeEvent(QExposeEvent *) { updateGeometryToAdapter(); }
void QtSimulateWindow::resizeEvent(QResizeEvent *) {
    updateGeometryToAdapter();
    if (m_adapterRaw) m_adapterRaw->PostResize(width(), height());
}

void QtSimulateWindow::updateGeometryToAdapter() {
    if (!m_adapterRaw) return;
    qreal dpr = devicePixelRatio();
    int w = width(), h = height();
    int fbw = int(w * dpr), fbh = int(h * dpr);
    QScreen *scr = screen();
    double dpi = scr ? scr->logicalDotsPerInch() : 96.0;
    m_adapterRaw->SetWindowGeometry(w, h, fbw, fbh, dpi);
}

int QtSimulateWindow::qtMouseButtonToInternal(int btn) const {
    if (btn == Qt::LeftButton)   return 1;
    if (btn == Qt::RightButton)  return 2;
    if (btn == Qt::MiddleButton) return 3;
    return 0;
}

void QtSimulateWindow::updateModifiersFrom(int qtMods) {
    if (!m_adapterRaw) return;
    m_adapterRaw->SetModifiers(qtMods & Qt::ControlModifier,
                               qtMods & Qt::ShiftModifier,
                               qtMods & Qt::AltModifier);
}

void QtSimulateWindow::mousePressEvent(QMouseEvent *e) {
    if (!m_adapterRaw) return;
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = devicePixelRatio();
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 1,
                                  e->pos().x() * dpr, e->pos().y() * dpr);
    setKeyboardGrabEnabled(true);  // 确保拿到键盘焦点
}
void QtSimulateWindow::mouseReleaseEvent(QMouseEvent *e) {
    if (!m_adapterRaw) return;
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = devicePixelRatio();
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 0,
                                  e->pos().x() * dpr, e->pos().y() * dpr);
}
void QtSimulateWindow::mouseMoveEvent(QMouseEvent *e) {
    if (!m_adapterRaw) return;
    updateModifiersFrom(int(e->modifiers()));
    qreal dpr = devicePixelRatio();
    m_adapterRaw->PostMouseMove(e->pos().x() * dpr, e->pos().y() * dpr);
}
void QtSimulateWindow::wheelEvent(QWheelEvent *e) {
    if (!m_adapterRaw) return;
    QPoint d = e->angleDelta();
    // GLFW 的 yoffset 单位是"刻度"，每格 120
    m_adapterRaw->PostScroll(d.x() / 120.0, d.y() / 120.0);
}

// 把 Qt key 翻译成 mjui 期望的键码（沿用 GLFW 的字符 / 功能键约定）
int QtSimulateWindow::qtKeyToMjui(int key) const {
    // 可打印字符直接传 ASCII
    if (key >= Qt::Key_Space && key <= Qt::Key_AsciiTilde) {
        // mjui 用大写字母 / 普通 ASCII；这里直接返回
        return key;
    }
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

void QtSimulateWindow::keyPressEvent(QKeyEvent *e) {
    if (!m_adapterRaw) return;
    updateModifiersFrom(int(e->modifiers()));
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 1);
}
void QtSimulateWindow::keyReleaseEvent(QKeyEvent *e) {
    if (!m_adapterRaw) return;
    updateModifiersFrom(int(e->modifiers()));
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 0);
}

// ----------------------------------------------------------------- 渲染线程 ----
void QtSimulateWindow::renderThreadMain() {
    // 在渲染线程里 makeCurrent
    m_ctx->moveToThread(QThread::currentThread());
    if (!m_ctx->makeCurrent(this)) {
        qWarning() << "QtSimulateWindow: makeCurrent failed";
        return;
    }

    // 构造 adapter / Simulate（在渲染线程内构造，匹配官方 main.cc 的顺序）
    static mjvCamera  cam;  mjv_defaultCamera(&cam);
    static mjvOption  opt;  mjv_defaultOption(&opt);
    static mjvPerturb pert; mjv_defaultPerturb(&pert);

    auto adapter_unique = std::make_unique<mjqt::QtPlatformUIAdapter>(this);
    m_adapterRaw = adapter_unique.get();

    m_sim = std::make_unique<mujoco::Simulate>(
        std::move(adapter_unique),
        &cam, &opt, &pert, /*is_passive=*/false);

    // 默认关闭 vsync；物理线程已有实时同步，渲染不需要帧锁速。
    // 用户可通过 UI 的 "Vertical Sync" 按钮手动开启。
    m_sim->vsync = 0;

    // 现在 adapter 已存在，把当前几何信息推过去
    QMetaObject::invokeMethod(this, [this] { updateGeometryToAdapter(); },
                              Qt::QueuedConnection);

    // 阻塞跑事件循环
    m_sim->RenderLoop();

    m_ctx->doneCurrent();
}

// ----------------------------------------------------------------- 物理线程 ----
namespace {
mjModel* loadModelFile(const QString &filename, mujoco::Simulate &sim) {
    char err[kErrorLength] = "";
    QByteArray utf8 = filename.toLocal8Bit();
    mjModel *m = nullptr;
    if (filename.endsWith(".mjb", Qt::CaseInsensitive)) {
        m = mj_loadModel(utf8.constData(), nullptr);
        if (!m) std::strncpy(err, "could not load binary model", sizeof(err)-1);
    } else if (filename.endsWith(".xml", Qt::CaseInsensitive)) {
        m = mj_loadXML(utf8.constData(), nullptr, err, sizeof(err));
    } else {
        mjSpec *spec = mj_parse(utf8.constData(), nullptr, nullptr, err, sizeof(err));
        if (spec) { m = mj_compile(spec, nullptr); mj_deleteSpec(spec); }
    }
    if (!m) {
        std::strncpy(sim.load_error, err, sizeof(sim.load_error)-1);
        std::printf("loadModel error: %s\n", err);
    }
    return m;
}
} // namespace

void QtSimulateWindow::physicsThreadMain() {
    // 等 Simulate 在渲染线程被构造好
    while (m_running.load() && !m_sim) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!m_sim) return;

    auto &sim = *m_sim;
    mjModel *m = nullptr;
    mjData  *d = nullptr;

    using Clock = mujoco::Simulate::Clock;
    std::chrono::time_point<Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim.exitrequest.load() && m_running.load()) {
        // 处理我方加载请求
        if (m_hasPendingLoad.exchange(false)) {
            QString file;
            { std::lock_guard<std::mutex> lk(m_pendingMtx); file = m_pendingFile; }
            sim.LoadMessage(file.toLocal8Bit().constData());
            mjModel *mnew = loadModelFile(file, sim);
            mjData  *dnew = mnew ? mj_makeData(mnew) : nullptr;
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

        // UI 触发的加载（Reload 按钮等）
        if (sim.uiloadrequest.load()) {
            sim.uiloadrequest.fetch_sub(1);
            sim.LoadMessage(sim.filename);
            mjModel *mnew = loadModelFile(QString::fromLocal8Bit(sim.filename), sim);
            mjData  *dnew = mnew ? mj_makeData(mnew) : nullptr;
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
            const auto startCPU  = Clock::now();
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;
            double slowdown   = 100.0 / sim.percentRealTime[sim.real_time_index];
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
