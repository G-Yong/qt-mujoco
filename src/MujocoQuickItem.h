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
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "QtPlatformUIAdapter.h"

class QOpenGLContext;
class QOffscreenSurface;

namespace mujoco { class Simulate; }

class MujocoQuickItem : public QQuickFramebufferObject, public mjqt::IMujocoHost {
    Q_OBJECT
    Q_PROPERTY(QString xmlPath READ xmlPath WRITE setXmlPath NOTIFY xmlPathChanged)
public:
    explicit MujocoQuickItem(QQuickItem *parent = nullptr);
    ~MujocoQuickItem() override;

    Renderer* createRenderer() const override;

    // QML / C++ 接口
    QString xmlPath() const { return m_xmlPath; }
    void    setXmlPath(const QString& path);

    Q_INVOKABLE void start(const QString& filename = QString());
    Q_INVOKABLE void stop();
    Q_INVOKABLE void loadModel(const QString& filename);

    // 给 Renderer 在 Quick 渲染线程读取的快照
    unsigned int currentSourceTexture() const;
    QSize        currentSourceSize() const;

    // Quick 渲染线程：采样完共享纹理后调用，解除 mujoco 渲染线程的等待。
    void notifyFrameConsumed();

    // IMujocoHost
    void onFrameRendered() override;
    void onSetTitle(const QString& title) override;
    void onToggleFullscreen() override;

signals:
    void xmlPathChanged();

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
    int  qtMouseButtonToInternal(int btn) const;
    int  qtKeyToMjui(int key) const;
    void updateModifiersFrom(int qtMods);

    QString m_xmlPath;

    QOpenGLContext*    m_ctx     = nullptr;
    QOffscreenSurface* m_surface = nullptr;

    std::unique_ptr<mujoco::Simulate>           m_sim;
    std::unique_ptr<mjqt::QtPlatformUIAdapter>  m_adapter;
    mjqt::QtPlatformUIAdapter*                  m_adapterRaw = nullptr;

    std::thread       m_renderThread;
    std::thread       m_physicsThread;
    std::atomic<bool> m_running {false};

    std::mutex        m_pendingMtx;
    QString           m_pendingFile;
    std::atomic<bool> m_hasPendingLoad {false};
};
