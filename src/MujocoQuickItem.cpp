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
#include <QFileInfo>
#include <QTemporaryFile>
#include <QDir>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace {
constexpr double  syncMisalign       = 0.1;
constexpr double  simRefreshFraction = 0.7;
constexpr int     kErrorLength       = 1024;
using Seconds = std::chrono::duration<double>;

int boolToInt(bool value) { return value ? 1 : 0; }

bool isValidIndex(int index, int count) {
    return index >= 0 && index < count;
}

int bitFlagIndex(int bit, int count) {
    if (bit <= 0) return -1;
    for (int i = 0; i < count; ++i) {
        if (bit == (1 << i)) return i;
    }
    return -1;
}
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
        // 我们只需要颜色 attachment：blit 不需要深度 / 模板，
        // 去掉可以节省大窗口下的显存带宽。
        fmt.setAttachment(QOpenGLFramebufferObject::NoAttachment);
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

        // 告知 mujoco 渲染线程：共享纹理已被取走，可以生成下一帧。
        m_item->notifyFrameConsumed();
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
    qRegisterMetaType<JointInfo>();
    qRegisterMetaType<ContactInfo>();
    setMirrorVertically(true); // mjr 是 OpenGL bottom-up，Quick 绘制时翻一下
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptHoverEvents(true);
    setFlag(ItemHasContents, true);
    setActiveFocusOnTab(true);
}

MujocoQuickItem::~MujocoQuickItem() { closeScene(); }

QQuickFramebufferObject::Renderer* MujocoQuickItem::createRenderer() const {
    return new MujocoFboRenderer();
}

void MujocoQuickItem::setXmlPath(const QString& path) {
    if (m_xmlPath == path) return;
    m_xmlPath = path;
    emit xmlPathChanged();
    if (m_running.load()) loadScene(path);
}

unsigned int MujocoQuickItem::currentSourceTexture() const {
    return m_adapterRaw ? m_adapterRaw->offscreenColorTexture() : 0u;
}
QSize MujocoQuickItem::currentSourceSize() const {
    if (!m_adapterRaw) return {};
    return {m_adapterRaw->offscreenWidth(), m_adapterRaw->offscreenHeight()};
}

void MujocoQuickItem::notifyFrameConsumed() {
    if (m_adapterRaw) m_adapterRaw->NotifyConsumed();
}

// ----------------------------------------------------------------- 场景生命周期 ----
void MujocoQuickItem::setLastError(const QString& err) {
    std::lock_guard<std::mutex> lk(m_errorMtx);
    m_lastError = err;
}

QString MujocoQuickItem::lastError() const {
    std::lock_guard<std::mutex> lk(m_errorMtx);
    return m_lastError;
}

bool MujocoQuickItem::ensureBackendStarted() {
    if (m_running.exchange(true)) {
        return true; // 已启动
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
        const QString err = QStringLiteral("GL context create failed");
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        delete m_ctx; m_ctx = nullptr;
        if (m_surface) { m_surface->destroy(); delete m_surface; m_surface = nullptr; }
        m_running.store(false);
        return false;
    }

    // 推到渲染线程（通过先释放线程亲和性到 nullptr，再由渲染线程接手）
    m_ctx->moveToThread(nullptr);
    m_surface->moveToThread(nullptr);

    updateGeometryToAdapter();

    m_renderThread  = std::thread(&MujocoQuickItem::renderThreadMain,  this);
    m_physicsThread = std::thread(&MujocoQuickItem::physicsThreadMain, this);

    return true;
}

void MujocoQuickItem::closeScene() {
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

    // 清理待加载请求和临时文件
    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile.clear();
    }
    m_hasPendingLoad.store(false);
    m_tempSceneFile.reset();
}

bool MujocoQuickItem::loadScene(const QString& filename) {
    QFileInfo checkFile(filename);
    if (filename.isEmpty() || !checkFile.exists() || !checkFile.isFile()) {
        const QString err = QStringLiteral("Scene file does not exist: %1").arg(filename);
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    if (!ensureBackendStarted()) {
        emit sceneLoadFailed(lastError());
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = filename;
    }
    m_hasPendingLoad.store(true);
    return true;
}

bool MujocoQuickItem::loadSceneFromData(const QByteArray& data, const QString& format) {
    if (data.isEmpty()) {
        const QString err = QStringLiteral("Scene data is empty");
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    QString suffix = format.trimmed().toLower();
    if (suffix.startsWith(QLatin1Char('.'))) suffix.remove(0, 1);
    if (suffix != QLatin1String("xml") && suffix != QLatin1String("mjb")) {
        const QString err = QStringLiteral("Unsupported scene format: %1 (expected 'xml' or 'mjb')").arg(format);
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    // 写入临时文件 (保留实例存活，以便 mujoco 异步加载期间文件不会被删除)
    auto tmp = std::unique_ptr<QTemporaryFile>(new QTemporaryFile(
        QDir::tempPath() + QStringLiteral("/mujoco_scene_XXXXXX.") + suffix));
    tmp->setAutoRemove(true);
    if (!tmp->open()) {
        const QString err = QStringLiteral("Failed to create temporary scene file: %1").arg(tmp->errorString());
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }
    if (tmp->write(data) != data.size()) {
        const QString err = QStringLiteral("Failed to write scene data to temporary file: %1").arg(tmp->errorString());
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }
    tmp->flush();
    const QString tmpPath = tmp->fileName();
    tmp->close();

    if (!ensureBackendStarted()) {
        emit sceneLoadFailed(lastError());
        return false;
    }

    // 保活：新临时文件覆盖之前的，物理线程加载完成后旧文件可被回收。
    m_tempSceneFile = std::move(tmp);

    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = tmpPath;
    }
    m_hasPendingLoad.store(true);
    return true;
}

void MujocoQuickItem::withSimulation(std::function<void(const mjModel*, mjData*)> callback) const {
    if (!m_sim) return;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    if (!m_sim->m_ || !m_sim->d_) return;
    callback(m_sim->m_, m_sim->d_);
}

void MujocoQuickItem::withMutableSimulation(std::function<void(mjModel*, mjData*)> callback) {
    if (!m_sim) return;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    if (!m_sim->m_ || !m_sim->d_) return;
    callback(m_sim->m_, m_sim->d_);
    lk.unlock();
    requestRenderUpdate();
}

void MujocoQuickItem::requestRenderUpdate() {
    if (QThread::currentThread() == thread()) {
        update();
    } else {
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    }
}

void MujocoQuickItem::applyBooleanPropertiesTo(mujoco::Simulate& sim) {
    sim.run = boolToInt(m_simulationRunning.load());
    sim.help = boolToInt(m_helpVisible.load());
    sim.info = boolToInt(m_infoVisible.load());
    sim.profiler = boolToInt(m_profilerVisible.load());
    sim.sensor = boolToInt(m_sensorVisible.load());
    sim.pause_update = boolToInt(m_pauseUpdateEnabled.load());
    sim.busywait = boolToInt(m_busyWaitEnabled.load());
    sim.ui0_enable = boolToInt(m_leftUiVisible.load());
    sim.ui1_enable = boolToInt(m_rightUiVisible.load());
    sim.status_overlay = boolToInt(m_statusOverlayVisible.load());

    const int fullscreen = boolToInt(m_fullscreenRequested.load());
    if (sim.fullscreen != fullscreen && sim.platform_ui) {
        sim.platform_ui->ToggleFullscreen();
    }
    sim.fullscreen = fullscreen;

    sim.vsync = boolToInt(m_vSyncEnabled.load());
    if (sim.platform_ui) sim.platform_ui->SetVSync(sim.vsync);

    sim.pending_.ui_update_simulation = true;
    sim.pending_.ui_update_rendering = true;
}

bool MujocoQuickItem::withSimulateLocked(const std::function<void(mujoco::Simulate&)>& callback) {
    if (!m_sim) return false;
    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        callback(*m_sim);
    }
    requestRenderUpdate();
    return true;
}

void MujocoQuickItem::setSimulationRunning(bool running) {
    if (m_simulationRunning.exchange(running) == running) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.run = boolToInt(running);
        if (sim.run) {
            sim.scrub_index = 0;
            sim.pert.active = 0;
        }
        sim.pending_.ui_update_simulation = true;
    });
    emit simulationRunningChanged();
}

bool MujocoQuickItem::toggleSimulationRunning() {
    setSimulationRunning(!simulationRunning());
    return true;
}

bool MujocoQuickItem::stepSimulationForward() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || sim.run) return;
        if (sim.scrub_index < 0) {
            sim.scrub_index++;
            sim.pending_.load_from_history = true;
            sim.pending_.ui_update_simulation = true;
        } else {
            mj_step(sim.m_, sim.d_);
            sim.AddToHistory();
        }
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::stepSimulationBackward() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_) return;
        sim.run = 0;
        sim.scrub_index = mjMAX(sim.scrub_index - 1, 1 - sim.nhistory_);
        sim.pending_.load_from_history = true;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    if (applied && m_simulationRunning.exchange(false)) {
        emit simulationRunningChanged();
    }
    return applied;
}

bool MujocoQuickItem::resetSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_) return;
        sim.pending_.reset = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::zeroControls() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_) return;
        sim.pending_.zero_ctrl = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setKeyframeIndex(int index) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !isValidIndex(index, sim.nkey_)) return;
        sim.key = index;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::loadKeyframe() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || !isValidIndex(sim.key, sim.nkey_)) return;
        sim.pending_.load_key = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveKeyframe() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || !isValidIndex(sim.key, sim.nkey_)) return;
        sim.pending_.save_key = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveSceneAsXml(const QString& filename) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.save_xml = filename.toStdString();
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveSceneAsMjb(const QString& filename) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.save_mjb = filename.toStdString();
        applied = true;
    });
    return applied;
}

// ---------------------------------------------------------------------------
// 关节查询与控制
// ---------------------------------------------------------------------------

static constexpr int kJntQposDim[] = {7, 4, 1, 1}; // free, ball, slide, hinge
static const char* const kJntTypeName[] = {"free", "ball", "slide", "hinge"};

// ---------------------------------------------------------------------------
// 接触快照构建：在 sim.mtx 锁内调用，读取当前帧所有接触并转成 ContactInfo 列表。
// ---------------------------------------------------------------------------
static QList<ContactInfo> buildContactSnapshot(mjModel* m, mjData* d)
{
    QList<ContactInfo> result;
    if (!m || !d || d->ncon <= 0) return result;
    result.reserve(d->ncon);

    auto nameOf = [m](int objType, int id) -> QString {
        if (id < 0) return QString();
        const char* n = mj_id2name(m, objType, id);
        return (n && n[0] != '\0') ? QString::fromUtf8(n)
                                   : QStringLiteral("#%1").arg(id);
    };

    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& c = d->contact[i];
        ContactInfo info;
        info.geom0Id = c.geom[0];
        info.geom1Id = c.geom[1];
        info.body0Id = (c.geom[0] >= 0) ? m->geom_bodyid[c.geom[0]] : -1;
        info.body1Id = (c.geom[1] >= 0) ? m->geom_bodyid[c.geom[1]] : -1;
        info.geom0Name = nameOf(mjOBJ_GEOM, c.geom[0]);
        info.geom1Name = nameOf(mjOBJ_GEOM, c.geom[1]);
        info.body0Name = nameOf(mjOBJ_BODY, info.body0Id);
        info.body1Name = nameOf(mjOBJ_BODY, info.body1Id);
        info.dist        = static_cast<double>(c.dist);
        info.active      = (c.exclude == 0) && (c.efc_address >= 0);
        info.penetrating = (c.dist < 0);
        // 法向接触力（mj_contactForce 返回 6D 质心力，force[0] 为法向分量）
        mjtNum force[6] = {};
        mj_contactForce(m, d, i, force);
        info.normalForce = static_cast<double>(force[0]);
        info.position = QVector3D(static_cast<float>(c.pos[0]),
                                  static_cast<float>(c.pos[1]),
                                  static_cast<float>(c.pos[2]));
        // 接触帧首行为接触法向（row-major，3x3）
        info.normal = QVector3D(static_cast<float>(c.frame[0]),
                                static_cast<float>(c.frame[1]),
                                static_cast<float>(c.frame[2]));
        result.append(std::move(info));
    }
    return result;
}

int MujocoQuickItem::jointCount() const
{
    int count = 0;
    withSimulation([&](const mjModel* m, mjData*) {
        count = static_cast<int>(m->njnt);
    });
    return count;
}

JointInfo MujocoQuickItem::jointInfo(int index) const
{
    JointInfo result;
    withSimulation([&](const mjModel* m, mjData*) {
        if (!isValidIndex(index, static_cast<int>(m->njnt))) return;
        int type = m->jnt_type[index];
        const char* rawName = mj_id2name(m, mjOBJ_JOINT, index);
        result.name      = rawName ? QString::fromUtf8(rawName)
                                   : QStringLiteral("joint_%1").arg(index);
        result.type      = type;
        result.typeName  = QString::fromLatin1(kJntTypeName[type]);
        result.qposDim   = kJntQposDim[type];
        result.limited   = (m->jnt_limited[index] != 0);
        result.rangeMin  = result.limited ? m->jnt_range[2 * index]     : 0.0;
        result.rangeMax  = result.limited ? m->jnt_range[2 * index + 1] : 0.0;
        result.stiffness = m->jnt_stiffness[index];
        result.qposadr   = m->jnt_qposadr[index];
    });
    return result;
}

QVariantList MujocoQuickItem::jointPosition(int index) const
{
    QVariantList result;
    withSimulation([&](const mjModel* m, mjData* d) {
        if (!isValidIndex(index, static_cast<int>(m->njnt))) return;
        int dim = kJntQposDim[m->jnt_type[index]];
        int adr = m->jnt_qposadr[index];
        for (int k = 0; k < dim; ++k)
            result.append(d->qpos[adr + k]);
    });
    return result;
}

bool MujocoQuickItem::setJointPosition(int index, const QVariantList& values)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int dim = kJntQposDim[sim.m_->jnt_type[index]];
        if (values.size() != dim) return;
        int adr = sim.m_->jnt_qposadr[index];
        for (int k = 0; k < dim; ++k)
            sim.qpos_[adr + k] = values[k].toDouble();
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointValue(int index, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int type = sim.m_->jnt_type[index];
        if (type != mjJNT_SLIDE && type != mjJNT_HINGE) return;
        sim.qpos_[sim.m_->jnt_qposadr[index]] = value;
        applied = true;
    });
    return applied;
}

// ---------------------------------------------------------------------------
// 碰撞检测查询
// ---------------------------------------------------------------------------

int MujocoQuickItem::contactCount() const
{
    return m_contactSnapshot.size();
}

ContactInfo MujocoQuickItem::contact(int index) const
{
    if (index < 0 || index >= m_contactSnapshot.size()) return ContactInfo{};
    return m_contactSnapshot.at(index);
}

QVariantList MujocoQuickItem::contacts() const
{
    QVariantList result;
    result.reserve(m_contactSnapshot.size());
    for (const ContactInfo& c : m_contactSnapshot)
        result.append(QVariant::fromValue(c));
    return result;
}

bool MujocoQuickItem::setControlNoise(double scale, double rate) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.ctrl_noise_std = scale;
        sim.ctrl_noise_rate = rate;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setRealTimeSpeedIndex(int index) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const int count = int(sizeof(mujoco::Simulate::percentRealTime) /
                              sizeof(mujoco::Simulate::percentRealTime[0]));
        if (sim.is_passive_ || !isValidIndex(index, count)) return;
        sim.real_time_index = index;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::speedUpSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || sim.real_time_index <= 0) return;
        sim.real_time_index--;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::slowDownSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const int count = int(sizeof(mujoco::Simulate::percentRealTime) /
                              sizeof(mujoco::Simulate::percentRealTime[0]));
        if (sim.is_passive_ || sim.real_time_index >= count - 1) return;
        sim.real_time_index++;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFreeCamera() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.cam.type = mjCAMERA_FREE;
        sim.camera = 0;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setTrackingCamera(int bodyId) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (!model) return;
        int target = bodyId >= 0 ? bodyId : sim.pert.select;
        if (target <= 0 || target >= model->nbody) return;
        sim.cam.type = mjCAMERA_TRACKING;
        sim.cam.trackbodyid = target;
        sim.cam.fixedcamid = -1;
        sim.camera = 1;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFixedCamera(int cameraIndex) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(cameraIndex, sim.ncam_)) return;
        sim.cam.type = mjCAMERA_FIXED;
        sim.cam.fixedcamid = cameraIndex;
        sim.camera = cameraIndex + 2;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::cycleFixedCamera(int direction) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.ncam_ <= 0 || direction == 0) return;
        int current = sim.cam.type == mjCAMERA_FIXED ? sim.cam.fixedcamid : -1;
        int next = current + (direction > 0 ? 1 : -1);
        if (next < 0) next = sim.ncam_ - 1;
        if (next >= sim.ncam_) next = 0;
        sim.cam.type = mjCAMERA_FIXED;
        sim.cam.fixedcamid = next;
        sim.camera = next + 2;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::alignView() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.align = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFrameVisualization(int frame) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(frame, mjNFRAME)) return;
        sim.opt.frame = frame;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::cycleFrameVisualization() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.opt.frame = (sim.opt.frame + 1) % mjNFRAME;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setLabelVisualization(int label) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(label, mjNLABEL)) return;
        sim.opt.label = label;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

// 将标签模式循环切换到下一种（超过 mjNLABEL 后回绕到 0）。
// 无需模型已加载，即使在加载前调用也会更新选项。
bool MujocoQuickItem::cycleLabelVisualization() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.opt.label = (sim.opt.label + 1) % mjNLABEL;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setVisualizationFlag(int flag, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(flag, mjNVISFLAG)) return;
        sim.opt.flags[flag] = enabled;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setRenderingFlag(int flag, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(flag, mjNRNDFLAG)) return;
        sim.scn.flags[flag] = enabled;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setVisualGroupVisible(unsigned char* groups, int group, bool visible) {
    if (!groups || !isValidIndex(group, mjNGROUP)) return false;
    groups[group] = visible;
    return true;
}

bool MujocoQuickItem::setGeomGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.geomgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setSiteGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.sitegroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.jointgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setTendonGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.tendongroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setActuatorGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.actuatorgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setFlexGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.flexgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setSkinGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.skingroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setPhysicsDisableFlag(int disableBit, bool disabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        const int index = bitFlagIndex(disableBit, mjNDISABLE);
        if (!model || index < 0) return;
        sim.disable[index] = boolToInt(disabled);
        if (disabled) model->opt.disableflags |= disableBit;
        else model->opt.disableflags &= ~disableBit;
        sim.pending_.ui_update_physics = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setPhysicsEnableFlag(int enableBit, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        const int index = bitFlagIndex(enableBit, mjNENABLE);
        if (!model || index < 0) return;
        sim.enable[index] = boolToInt(enabled);
        if (enabled) model->opt.enableflags |= enableBit;
        else model->opt.enableflags &= ~enableBit;
        sim.pending_.ui_update_physics = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setActuatorGroupEnabled(int group, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (!model || !isValidIndex(group, mjNGROUP)) return;
        sim.enableactuator[group] = boolToInt(enabled);
        if (enabled) model->opt.disableactuator &= ~(1 << group);
        else model->opt.disableactuator |= (1 << group);
        sim.pending_.ui_update_physics = true;
        sim.pending_.ui_remake_ctrl = true;
        applied = true;
    });
    return applied;
}

void MujocoQuickItem::setHelpVisible(bool visible) {
    if (m_helpVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.help = boolToInt(visible); });
    emit helpVisibleChanged();
}

void MujocoQuickItem::setInfoVisible(bool visible) {
    if (m_infoVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.info = boolToInt(visible); });
    emit infoVisibleChanged();
}

void MujocoQuickItem::setProfilerVisible(bool visible) {
    if (m_profilerVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.profiler = boolToInt(visible); });
    emit profilerVisibleChanged();
}

void MujocoQuickItem::setSensorVisible(bool visible) {
    if (m_sensorVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.sensor = boolToInt(visible); });
    emit sensorVisibleChanged();
}

void MujocoQuickItem::setPauseUpdateEnabled(bool enabled) {
    if (m_pauseUpdateEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.pause_update = boolToInt(enabled); });
    emit pauseUpdateEnabledChanged();
}

void MujocoQuickItem::setFullscreenRequested(bool enabled) {
    if (m_fullscreenRequested.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.fullscreen != boolToInt(enabled) && sim.platform_ui) {
            sim.platform_ui->ToggleFullscreen();
        }
        sim.fullscreen = boolToInt(enabled);
    });
    emit fullscreenRequestedChanged();
}

void MujocoQuickItem::setVSyncEnabled(bool enabled) {
    if (m_vSyncEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.vsync = boolToInt(enabled);
        if (sim.platform_ui) sim.platform_ui->SetVSync(enabled);
    });
    emit vSyncEnabledChanged();
}

void MujocoQuickItem::setBusyWaitEnabled(bool enabled) {
    if (m_busyWaitEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.busywait = boolToInt(enabled); });
    emit busyWaitEnabledChanged();
}

void MujocoQuickItem::setLeftUiVisible(bool visible) {
    if (m_leftUiVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.ui0_enable = boolToInt(visible); });
    emit leftUiVisibleChanged();
}

void MujocoQuickItem::setRightUiVisible(bool visible) {
    if (m_rightUiVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.ui1_enable = boolToInt(visible); });
    emit rightUiVisibleChanged();
}

void MujocoQuickItem::setStatusOverlayVisible(bool visible) {
    if (m_statusOverlayVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.status_overlay = boolToInt(visible); });
    emit statusOverlayVisibleChanged();
}

// ----------------------------------------------------------------- IMujocoHost
void MujocoQuickItem::onFrameRendered() {
    // 在 mujoco 渲染线程被调用，转发到 GUI 线程触发 update()
    const QString statusText = m_sim ? QString::fromUtf8(m_sim->status_overlay_text) : QString();

    // 在锁内采样当前帧接触信息（sim.mtx 是 recursive_mutex，渲染线程可能已持锁）
    QList<ContactInfo> contacts;
    if (m_sim) {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        if (m_sim->m_ && m_sim->d_)
            contacts = buildContactSnapshot(m_sim->m_, m_sim->d_);
    }

    QMetaObject::invokeMethod(this, [this, statusText, contacts = std::move(contacts)] {
        if (m_statusOverlayText != statusText) {
            m_statusOverlayText = statusText;
            emit statusOverlayTextChanged();
        }
        m_contactSnapshot = std::move(contacts);
        emit contactsChanged();
        update();
    }, Qt::QueuedConnection);
}
void MujocoQuickItem::onSetTitle(const QString& t) {
    // 注意：作为嵌入式 QQuickItem，不应擅自修改宿主窗口标题
    // （否则会出现 QML Window 标题被改成 "MuJoCo : <model>" 的问题）。
    // 仅通过信号把标题对外暴露，由上层 QML/C++ 自行决定如何使用。
    QMetaObject::invokeMethod(this, [this, t] {
        if (m_modelTitle == t) return;
        m_modelTitle = t;
        emit modelTitleChanged();
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

    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        applyBooleanPropertiesTo(*m_sim);
    }

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
                lk.unlock();
                setLastError(QString());
                const QString src = file;
                QMetaObject::invokeMethod(this, [this, src]{ emit sceneLoaded(src); },
                                          Qt::QueuedConnection);
            } else {
                sim.LoadMessageClear();
                const QString reason = QString::fromLocal8Bit(sim.load_error);
                setLastError(reason);
                QMetaObject::invokeMethod(this, [this, reason]{ emit sceneLoadFailed(reason); },
                                          Qt::QueuedConnection);
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
                lk.unlock();
                setLastError(QString());
                const QString src = QString::fromLocal8Bit(sim.filename);
                QMetaObject::invokeMethod(this, [this, src]{ emit sceneLoaded(src); },
                                          Qt::QueuedConnection);
            } else {
                sim.LoadMessageClear();
                const QString reason = QString::fromLocal8Bit(sim.load_error);
                setLastError(reason);
                QMetaObject::invokeMethod(this, [this, reason]{ emit sceneLoadFailed(reason); },
                                          Qt::QueuedConnection);
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
