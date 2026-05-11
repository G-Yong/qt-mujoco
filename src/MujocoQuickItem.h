#pragma once
// ---------------------------------------------------------------------------
// MujocoQuickItem
//
// 基于 QQuickFramebufferObject 的 QML 集成点，把 mujoco::Simulate 的渲染
// 输出搬到 Qt Quick scenegraph：
//
//   Mujoco 渲染线程 (QOffscreenSurface)
//      └─ Simulate::RenderLoop()
//             └─ mjr_render → con.offFBO (multisample)
//             └─ SwapBuffers (我们改成) blit 到 con.offFBO_r → 通知主线程
//   ↓ 跨 context 共享纹理 (Qt::AA_ShareOpenGLContexts)
//   Qt Quick scenegraph 渲染线程
//      └─ MujocoFboRenderer::render()
//             └─ glBlitFramebuffer 把共享纹理拷贝到 Quick 提供的 FBO
//
// 使用：
//   - QML: import 后实例化 MujocoView，通过 xmlPath 或 loadModel 加载模型
//   - C++: 也可作为 QQuickItem 嵌入；Widget 工程通过 QQuickWidget 集成
// ---------------------------------------------------------------------------

#include <QQuickFramebufferObject>
#include <QString>
#include <QSize>
#include <QVariant>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "QtPlatformUIAdapter.h"

class QOpenGLContext;
class QOffscreenSurface;
class QTemporaryFile;

#include "simulationtypes.h"

namespace mujoco { class Simulate; }

class MujocoQuickItem : public QQuickFramebufferObject, public mjqt::IMujocoHost {
    Q_OBJECT
    Q_PROPERTY(QString xmlPath READ xmlPath WRITE setXmlPath NOTIFY xmlPathChanged)
    Q_PROPERTY(bool simulationRunning READ simulationRunning WRITE setSimulationRunning NOTIFY simulationRunningChanged)
    Q_PROPERTY(bool helpVisible READ helpVisible WRITE setHelpVisible NOTIFY helpVisibleChanged)
    Q_PROPERTY(bool infoVisible READ infoVisible WRITE setInfoVisible NOTIFY infoVisibleChanged)
    Q_PROPERTY(bool profilerVisible READ profilerVisible WRITE setProfilerVisible NOTIFY profilerVisibleChanged)
    Q_PROPERTY(bool sensorVisible READ sensorVisible WRITE setSensorVisible NOTIFY sensorVisibleChanged)
    Q_PROPERTY(bool pauseUpdateEnabled READ pauseUpdateEnabled WRITE setPauseUpdateEnabled NOTIFY pauseUpdateEnabledChanged)
    Q_PROPERTY(bool fullscreenRequested READ fullscreenRequested WRITE setFullscreenRequested NOTIFY fullscreenRequestedChanged)
    Q_PROPERTY(bool vSyncEnabled READ vSyncEnabled WRITE setVSyncEnabled NOTIFY vSyncEnabledChanged)
    Q_PROPERTY(bool busyWaitEnabled READ busyWaitEnabled WRITE setBusyWaitEnabled NOTIFY busyWaitEnabledChanged)
    Q_PROPERTY(bool leftUiVisible READ leftUiVisible WRITE setLeftUiVisible NOTIFY leftUiVisibleChanged)
    Q_PROPERTY(bool rightUiVisible READ rightUiVisible WRITE setRightUiVisible NOTIFY rightUiVisibleChanged)
    Q_PROPERTY(bool statusOverlayVisible READ statusOverlayVisible WRITE setStatusOverlayVisible NOTIFY statusOverlayVisibleChanged)
    Q_PROPERTY(QString statusOverlayText READ statusOverlayText NOTIFY statusOverlayTextChanged)
    Q_PROPERTY(QString modelTitle READ modelTitle NOTIFY modelTitleChanged)
    Q_PROPERTY(int contactCount READ contactCount NOTIFY contactsChanged)
public:
    explicit MujocoQuickItem(QQuickItem *parent = nullptr);
    ~MujocoQuickItem() override;

    Renderer* createRenderer() const override;

    // QML / C++ 接口
    QString xmlPath() const { return m_xmlPath; }
    void    setXmlPath(const QString& path);

    // 场景生命周期 ---------------------------------------------------------
    // loadScene: 从磁盘加载 .xml / .mjb 场景；
    //   (GL context / 渲染线程 / 物理线程)。返回值表示请求是否被接受 (同步
    //   阶段的输入校验)，实际加载结果通过 sceneLoaded / sceneLoadFailed
    //   信号异步通知。
    Q_INVOKABLE bool loadScene(const QString& filename);
    // loadSceneFromData: 从内存缓冲加载场景。format 指定数据格式，可取
    //   "xml" 或 "mjb"。内部通过临时文件方式转交给 mujoco 加载流程。
    Q_INVOKABLE bool loadSceneFromData(const QByteArray& data,
                                       const QString& format = QStringLiteral("xml"));
    // closeScene: 关闭当前场景并释放渲染/物理线程及相关资源。
    Q_INVOKABLE void closeScene();
    // lastError: 最近一次加载失败的错误信息 (中文/英文均可，由底层提供)。
    Q_INVOKABLE QString lastError() const;
    // saveModelAsXml: 将当前已编译的模型以 MJCF XML 格式写入 filename。
    //   注意：外部资源（mesh/纹理）不嵌入，仅保留引用路径。
    //   需要模型已加载；通过 pending 机制在渲染循环下一帧执行。
    Q_INVOKABLE bool saveSceneAsXml(const QString& filename);
    // saveSceneAsMjb: 将当前已编译的模型以 MuJoCo 二进制格式（.mjb）写入 filename。
    //   完全自包含：内嵌所有 mesh/纹理数据，无需原始资源文件即可重新加载。
    Q_INVOKABLE bool saveSceneAsMjb(const QString& filename);

    // ------------------------------------------------------------------
    // 关节查询与控制接口
    // ------------------------------------------------------------------

    // 返回当前场景的关节数量；场景未加载时返回 0。
    Q_INVOKABLE int       jointCount() const;
    // 返回第 index 个关节的固有属性；index 越界时返回默认构造的空结构。
    Q_INVOKABLE JointInfo jointInfo(int index) const;
    // 读取第 index 个关节的当前 qpos 值（列表长度 = qposDim）。
    // hinge/slide 关节列表长度为 1，ball 为 4，free 为 7。
    // 场景未加载或 index 越界时返回空列表。
    Q_INVOKABLE QVariantList jointPosition(int index) const;
    // 设置第 index 个关节的 qpos 值。values 长度须与 qposDim 匹配。
    // 对 hinge/slide 关节可直接传 [angle] 或 [distance]。
    // 返回 false 表示场景未加载、index 越界或 values 长度不匹配。
    Q_INVOKABLE bool setJointPosition(int index, const QVariantList& values);
    // 单自由度快捷接口，仅适用于 hinge / slide 关节。
    // 对 free / ball 关节调用会返回 false。
    Q_INVOKABLE bool setJointValue(int index, double value);

    Q_INVOKABLE bool toggleSimulationRunning();
    Q_INVOKABLE bool stepSimulationForward();
    Q_INVOKABLE bool stepSimulationBackward();
    Q_INVOKABLE bool resetSimulation();
    Q_INVOKABLE bool zeroControls();
    Q_INVOKABLE bool setKeyframeIndex(int index);
    Q_INVOKABLE bool loadKeyframe();
    Q_INVOKABLE bool saveKeyframe();
    Q_INVOKABLE bool setControlNoise(double scale, double rate);
    Q_INVOKABLE bool setRealTimeSpeedIndex(int index);
    Q_INVOKABLE bool speedUpSimulation();
    Q_INVOKABLE bool slowDownSimulation();

    Q_INVOKABLE bool setFreeCamera();
    Q_INVOKABLE bool setTrackingCamera(int bodyId = -1);
    Q_INVOKABLE bool setFixedCamera(int cameraIndex);
    Q_INVOKABLE bool cycleFixedCamera(int direction = 1);
    Q_INVOKABLE bool alignView();

    Q_INVOKABLE bool setFrameVisualization(int frame);
    Q_INVOKABLE bool cycleFrameVisualization();

    // 设置场景中物体的标签显示模式（对应 mjOption::label）。
    // label 取值为 mjtLabel 枚举（定义于 mujoco/mjvisualize.h）：
    //   mjLABEL_NONE        = 0   — 不显示任何标签
    //   mjLABEL_BODY              — 显示 body 名称
    //   mjLABEL_JOINT             — 显示 joint 名称
    //   mjLABEL_GEOM              — 显示 geom 名称
    //   mjLABEL_SITE              — 显示 site 名称
    //   mjLABEL_CAMERA            — 显示 camera 名称
    //   mjLABEL_LIGHT             — 显示 light 名称
    //   mjLABEL_TENDON            — 显示 tendon 名称
    //   mjLABEL_ACTUATOR          — 显示 actuator 名称
    //   mjLABEL_CONSTRAINT        — 显示 constraint 名称
    //   mjLABEL_FLEX              — 显示 flex 名称
    //   mjLABEL_SKIN              — 显示 skin 名称
    //   mjLABEL_SELECTION         — 显示当前选中对象名称
    //   mjLABEL_SELPNT            — 显示选中点坐标
    //   mjLABEL_CONTACTPOINT      — 显示接触点信息
    //   mjLABEL_CONTACTFORCE      — 显示接触力大小
    //   mjLABEL_ISLAND            — 显示约束岛 ID
    // 返回值：label 合法且模拟已加载时返回 true，否则返回 false（不做任何修改）。
    Q_INVOKABLE bool setLabelVisualization(int label);
    Q_INVOKABLE bool cycleLabelVisualization();

    Q_INVOKABLE bool setVisualizationFlag(int flag, bool enabled);
    Q_INVOKABLE bool setRenderingFlag(int flag, bool enabled);
    Q_INVOKABLE bool setGeomGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setSiteGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setJointGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setTendonGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setActuatorGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setFlexGroupVisible(int group, bool visible);
    Q_INVOKABLE bool setSkinGroupVisible(int group, bool visible);

    Q_INVOKABLE bool setPhysicsDisableFlag(int disableBit, bool disabled);
    Q_INVOKABLE bool setPhysicsEnableFlag(int enableBit, bool enabled);
    Q_INVOKABLE bool setActuatorGroupEnabled(int group, bool enabled);

    // ------------------------------------------------------------------
    // 碰撞检测查询接口
    // ------------------------------------------------------------------

    // 返回当前帧快照中的接触数量。每帧在渲染回调中自动更新，线程安全。
    int contactCount() const;
    // 返回快照中第 index 个接触的详细信息；越界时返回默认构造值。
    Q_INVOKABLE ContactInfo  contact(int index) const;
    // 以 QVariantList 形式返回全部接触快照，供 QML 中遍历使用。
    Q_INVOKABLE QVariantList contacts() const;

    // 线程安全地访问仿真数据（在 sim.mtx 锁内执行 callback）。
    // callback 在物理/渲染线程之外的调用线程中执行，持锁期间
    // 可安全读写 mjModel / mjData，但不可长时间阻塞或在内部
    // 调用任何会再次加锁 sim.mtx 的方法。
    // 若仿真尚未就绪（m / d 为空），callback 不会被调用。
    void withSimulation(std::function<void(const mjModel*, mjData*)> callback) const;
    void withMutableSimulation(std::function<void(mjModel*, mjData*)> callback);

    // 给 Renderer 在 Quick 渲染线程读取的快照
    unsigned int currentSourceTexture() const;
    QSize        currentSourceSize() const;

    // Quick 渲染线程：采样完共享纹理后调用，解除 mujoco 渲染线程的等待。
    void notifyFrameConsumed();

    // IMujocoHost
    void onFrameRendered() override;
    void onSetTitle(const QString& title) override;
    void onToggleFullscreen() override;

public:
    bool simulationRunning() const { return m_simulationRunning.load(); }
    Q_INVOKABLE void setSimulationRunning(bool running);

    bool helpVisible() const { return m_helpVisible.load(); }
    bool infoVisible() const { return m_infoVisible.load(); }
    bool profilerVisible() const { return m_profilerVisible.load(); }
    bool sensorVisible() const { return m_sensorVisible.load(); }
    bool pauseUpdateEnabled() const { return m_pauseUpdateEnabled.load(); }
    bool fullscreenRequested() const { return m_fullscreenRequested.load(); }
    bool vSyncEnabled() const { return m_vSyncEnabled.load(); }
    bool busyWaitEnabled() const { return m_busyWaitEnabled.load(); }
    bool leftUiVisible() const { return m_leftUiVisible.load(); }
    bool rightUiVisible() const { return m_rightUiVisible.load(); }
    bool statusOverlayVisible() const { return m_statusOverlayVisible.load(); }
    QString statusOverlayText() const { return m_statusOverlayText; }
    QString modelTitle() const { return m_modelTitle; }

    Q_INVOKABLE void setHelpVisible(bool visible);
    Q_INVOKABLE void setInfoVisible(bool visible);
    Q_INVOKABLE void setProfilerVisible(bool visible);
    Q_INVOKABLE void setSensorVisible(bool visible);
    Q_INVOKABLE void setPauseUpdateEnabled(bool enabled);
    Q_INVOKABLE void setFullscreenRequested(bool enabled);
    Q_INVOKABLE void setVSyncEnabled(bool enabled);
    Q_INVOKABLE void setBusyWaitEnabled(bool enabled);
    Q_INVOKABLE void setLeftUiVisible(bool visible);
    Q_INVOKABLE void setRightUiVisible(bool visible);
    Q_INVOKABLE void setStatusOverlayVisible(bool visible);

signals:
    void xmlPathChanged();
    void simulationRunningChanged();
    void helpVisibleChanged();
    void infoVisibleChanged();
    void profilerVisibleChanged();
    void sensorVisibleChanged();
    void pauseUpdateEnabledChanged();
    void fullscreenRequestedChanged();
    void vSyncEnabledChanged();
    void busyWaitEnabledChanged();
    void leftUiVisibleChanged();
    void rightUiVisibleChanged();
    void statusOverlayVisibleChanged();
    void statusOverlayTextChanged();
    void modelTitleChanged();
    // 每帧在渲染回调后发出，携带最新接触快照（即使接触数量未变也会发出）。
    void contactsChanged();

    // 场景加载结果通知
    void sceneLoaded(const QString& source);
    void sceneLoadFailed(const QString& reason);

protected:
    void mousePressEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void hoverMoveEvent(QHoverEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void keyReleaseEvent(QKeyEvent*) override;
#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
    void geometryChange(const QRectF& newGeo, const QRectF& oldGeo) override;
#else
    void geometryChanged(const QRectF& newGeo, const QRectF& oldGeo) override;
#endif
    void itemChange(ItemChange, const ItemChangeData&) override;

private:
    void renderThreadMain();
    void physicsThreadMain();
    void updateGeometryToAdapter();
    void requestRenderUpdate();
    void applyBooleanPropertiesTo(mujoco::Simulate& sim);
    bool withSimulateLocked(const std::function<void(mujoco::Simulate&)>& callback);
    bool setVisualGroupVisible(mjtByte* groups, int group, bool visible);
    int  qtMouseButtonToInternal(int btn) const;
    int  qtKeyToMjui(int key) const;
    void updateModifiersFrom(int qtMods);

    // 启动后端 (GL context + 渲染/物理线程)。幂等：已启动时直接返回 true。
    bool ensureBackendStarted();
    // 设置最近一次错误信息 (线程安全)。
    void setLastError(const QString& err);

    QString m_xmlPath;

    QOpenGLContext*    m_ctx     = nullptr;
    QOffscreenSurface* m_surface = nullptr;

    std::unique_ptr<mujoco::Simulate>           m_sim;
    std::unique_ptr<mjqt::QtPlatformUIAdapter>  m_adapter;
    mjqt::QtPlatformUIAdapter*                  m_adapterRaw = nullptr;

    std::thread       m_renderThread;
    std::thread       m_physicsThread;
    std::atomic<bool> m_running {false};

    std::atomic<bool> m_simulationRunning {true};
    std::atomic<bool> m_helpVisible {false};
    std::atomic<bool> m_infoVisible {false};
    std::atomic<bool> m_profilerVisible {false};
    std::atomic<bool> m_sensorVisible {false};
    std::atomic<bool> m_pauseUpdateEnabled {false};
    std::atomic<bool> m_fullscreenRequested {false};
    std::atomic<bool> m_vSyncEnabled {false};
    std::atomic<bool> m_busyWaitEnabled {false};
    std::atomic<bool> m_leftUiVisible {true};
    std::atomic<bool> m_rightUiVisible {true};
    std::atomic<bool> m_statusOverlayVisible {true};

    QString           m_statusOverlayText;
    QString           m_modelTitle;

    std::mutex        m_pendingMtx;
    QString           m_pendingFile;
    std::atomic<bool> m_hasPendingLoad {false};

    mutable std::mutex m_errorMtx;
    QString            m_lastError;

    // loadSceneFromData 写入的临时文件，需保持存活直到下次加载或关闭。
    std::unique_ptr<QTemporaryFile> m_tempSceneFile;

    // 接触快照：由 onFrameRendered() 在主线程更新，contactCount()/contact() 读取。
    // 仅在主线程访问，无需额外锁。
    QList<ContactInfo> m_contactSnapshot;
};
