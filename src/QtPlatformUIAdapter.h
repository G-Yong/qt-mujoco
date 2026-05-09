#pragma once
// ---------------------------------------------------------------------------
// QtPlatformUIAdapter
//
// 实现 mujoco::PlatformUIAdapter，给官方 mujoco::Simulate 提供 Qt 后端。
//
// 与官方 GlfwAdapter 不同，本适配器不持有窗口/缓冲区交换：
//   - 渲染目标是 mjr 内置的 offscreen FBO（mjFB_OFFSCREEN）
//   - "SwapBuffers" 改为：把 multisample offFBO 解析到 offFBO_r，
//     然后通过回调通知宿主（QQuickFramebufferObject）重绘。
//
// 这样官方 simulate.cc 不需要任何修改 —— 它仍然像往常一样调用
// mjr_render / mjui_render / SwapBuffers，只是底层写入到我们提供的 FBO。
//
// 线程模型：
//   - Qt 主线程  ：收 QQuickItem 事件，调 Post*() 入队
//   - 渲染线程   ：QOffscreenSurface + 私有 QOpenGLContext，
//                  跑 Simulate::RenderLoop()，PollEvents() 出队回放
// ---------------------------------------------------------------------------

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <utility>

#include <QString>

#include "platform_ui_adapter.h"

namespace mjqt {

// 宿主接口：MujocoQuickItem 实现该接口，供适配器回调
class IMujocoHost {
public:
    virtual ~IMujocoHost() = default;
    // 渲染线程：每帧 mjr 渲染完成后调用，宿主据此触发 Quick scenegraph 重绘
    virtual void onFrameRendered() = 0;
    // Qt 主线程相关：仅做转发，宿主自行 invokeMethod
    virtual void onSetTitle(const QString& title) = 0;
    virtual void onToggleFullscreen() = 0;
};

class QtPlatformUIAdapter : public mujoco::PlatformUIAdapter {
public:
    explicit QtPlatformUIAdapter(IMujocoHost* host);
    ~QtPlatformUIAdapter() override;

    // ---- PlatformUIAdapter ----
    std::pair<double, double> GetCursorPosition() const override;
    double GetDisplayPixelsPerInch() const override;
    std::pair<int, int> GetFramebufferSize() const override;
    std::pair<int, int> GetWindowSize() const override;
    bool IsGPUAccelerated() const override;
    void PollEvents() override;
    void SetClipboardString(const char* text) override;
    void SetVSync(bool enabled) override;
    void SetWindowTitle(const char* title) override;
    bool ShouldCloseWindow() const override;
    void SwapBuffers() override;
    void ToggleFullscreen() override;

    // 拦截以切换 mjr 到 OFFSCREEN 模式 + 调整 offscreen FBO 尺寸
    bool RefreshMjrContext(const mjModel* m, int fontscale) override;
    bool EnsureContextSize() override;

    bool IsLeftMouseButtonPressed()   const override { return m_btnLeft.load();   }
    bool IsMiddleMouseButtonPressed() const override { return m_btnMiddle.load(); }
    bool IsRightMouseButtonPressed()  const override { return m_btnRight.load();  }

    bool IsAltKeyPressed()   const override { return m_modAlt.load();   }
    bool IsCtrlKeyPressed()  const override { return m_modCtrl.load();  }
    bool IsShiftKeyPressed() const override { return m_modShift.load(); }

    bool IsMouseButtonDownEvent(int act) const override { return act == 1; }
    bool IsKeyDownEvent(int act)         const override { return act == 1; }

    int        TranslateKeyCode(int key)        const override { return key; }
    mjtButton  TranslateMouseButton(int btn)    const override;

    // ---- Qt 主线程 → 渲染线程 ----（线程安全）
    // act: 1 = press/down, 0 = release/up
    void PostMouseButton(int qtButton, int act, double x, double y);
    void PostMouseMove(double x, double y);
    void PostScroll(double dx, double dy);
    void PostKey(int mjKey, int act);
    void PostResize(int w, int h);
    void PostClose();

    // 修饰键状态（Qt 主线程在事件前先更新）
    void SetModifiers(bool ctrl, bool shift, bool alt);

    // 窗口尺寸 / 像素比（Qt 主线程在 resize 时更新）
    void SetWindowGeometry(int win_w, int win_h, int fb_w, int fb_h, double dpi);

    // 渲染线程：返回当前 offscreen 已解析颜色纹理的 GL id（可能为 0）
    // 注意：这是我们自己分配的 GL 纹理（跨 context 可共享），
    //       而不是 mjr 的 con_.offColor_r（那是 renderbuffer，不能跨 context 共享，
    //       也不能当作 GL_TEXTURE_2D 采样）。
    unsigned int offscreenColorTexture() const { return m_sharedTex.load(); }
    int offscreenWidth()  const { return m_offW.load(); }
    int offscreenHeight() const { return m_offH.load(); }

    // 渲染线程：在 GL context 当前的情况下释放我们持有的 GL 资源
    void ReleaseSharedGL();

    // ---- 帧节拍（mujoco 渲染 vs Qt Quick 场景图渲染）----
    // Qt Quick 渲染线程在从 m_sharedTex 采样完成后调用，表示上一帧已被消费。
    void NotifyConsumed();
    // mujoco 渲染线程在 SwapBuffers 内调用；等待上一帧被消费后再进入下一帧，
    // 带超时以避免 Quick 不可见时冻住。

private:
    IMujocoHost* m_host;

    // 鼠标 / 修饰键状态
    std::atomic<bool>   m_btnLeft   {false};
    std::atomic<bool>   m_btnMiddle {false};
    std::atomic<bool>   m_btnRight  {false};
    std::atomic<bool>   m_modCtrl   {false};
    std::atomic<bool>   m_modShift  {false};
    std::atomic<bool>   m_modAlt    {false};

    // 光标
    mutable std::mutex      m_cursorMtx;
    double                  m_cursorX = 0;
    double                  m_cursorY = 0;

    // 当前 item 像素尺寸（Qt 主线程写、渲染线程读）
    std::atomic<int>    m_winW {1280};
    std::atomic<int>    m_winH {800};
    std::atomic<int>    m_fbW  {1280};
    std::atomic<int>    m_fbH  {800};
    std::atomic<double> m_dpi  {96.0};

    // 当前 mjr 内 offscreen FBO 的尺寸（渲染线程更新）
    std::atomic<int>          m_offW {0};
    std::atomic<int>          m_offH {0};

    // 我们自己创建的、可跨 context 共享的颜色纹理 + 配套 FBO；
    // mjr 渲染完成后我们把 multisample 的 con_.offFBO 解析到这里，
    // 再让 Qt Quick 渲染线程通过这个纹理采样。
    std::atomic<unsigned int> m_sharedTex {0};
    unsigned int              m_sharedFbo = 0;
    int                       m_sharedW = 0;
    int                       m_sharedH = 0;

    bool EnsureSharedTarget(int w, int h);

    std::atomic<bool>   m_shouldClose {false};

    // 帧节拍同步：mujoco 渲染线程在每交出一帧后等待 Quick 取走，
    // 避免以 GPU 极限速度不断覆写共享纹理、占用 Quick 合成资源。
    std::mutex                m_consumeMtx;
    std::condition_variable   m_consumeCv;
    bool                      m_frameConsumed = true;

    // 事件队列
    std::mutex                          m_qMtx;
    std::deque<std::function<void()>>   m_queue;
};

} // namespace mjqt
