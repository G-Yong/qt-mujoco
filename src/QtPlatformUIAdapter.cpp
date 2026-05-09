#include "QtPlatformUIAdapter.h"

#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>
#include <mujoco/mjrender.h>

#include <QGuiApplication>
#include <QClipboard>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>

namespace mjqt {

QtPlatformUIAdapter::QtPlatformUIAdapter(IMujocoHost* host) : m_host(host) {}
QtPlatformUIAdapter::~QtPlatformUIAdapter() {
    // GL 资源应该在 GL context 当前时释放；MujocoQuickItem::stop()
    // 会在 doneCurrent 之前调用 ReleaseSharedGL。这里只是收尾。
    FreeMjrContext();
}

bool QtPlatformUIAdapter::EnsureSharedTarget(int w, int h) {
    auto* glctx = QOpenGLContext::currentContext();
    if (!glctx || w <= 0 || h <= 0) return false;
    if (m_sharedTex.load() && m_sharedFbo && m_sharedW == w && m_sharedH == h) {
        return true;
    }

    QOpenGLExtraFunctions* gl = glctx->extraFunctions();

    unsigned int tex = m_sharedTex.load();
    if (!tex) {
        glctx->functions()->glGenTextures(1, &tex);
        m_sharedTex.store(tex);
    }
    glctx->functions()->glBindTexture(GL_TEXTURE_2D, tex);
    glctx->functions()->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0,
                                     GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glctx->functions()->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glctx->functions()->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glctx->functions()->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glctx->functions()->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glctx->functions()->glBindTexture(GL_TEXTURE_2D, 0);

    if (!m_sharedFbo) {
        gl->glGenFramebuffers(1, &m_sharedFbo);
    }
    GLint prev = 0;
    gl->glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev);
    gl->glBindFramebuffer(GL_FRAMEBUFFER, m_sharedFbo);
    gl->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, tex, 0);
    gl->glBindFramebuffer(GL_FRAMEBUFFER, prev);

    m_sharedW = w;
    m_sharedH = h;
    return true;
}

void QtPlatformUIAdapter::ReleaseSharedGL() {
    auto* glctx = QOpenGLContext::currentContext();
    if (!glctx) return;
    QOpenGLExtraFunctions* gl = glctx->extraFunctions();
    if (m_sharedFbo) {
        gl->glDeleteFramebuffers(1, &m_sharedFbo);
        m_sharedFbo = 0;
    }
    unsigned int tex = m_sharedTex.exchange(0);
    if (tex) {
        glctx->functions()->glDeleteTextures(1, &tex);
    }
    m_sharedW = m_sharedH = 0;
}

// ---------------- 基本 getter ----------------
std::pair<double,double> QtPlatformUIAdapter::GetCursorPosition() const {
    std::lock_guard<std::mutex> lk(m_cursorMtx);
    return {m_cursorX, m_cursorY};
}
double QtPlatformUIAdapter::GetDisplayPixelsPerInch() const   { return m_dpi.load(); }
std::pair<int,int> QtPlatformUIAdapter::GetFramebufferSize() const { return {m_fbW.load(),  m_fbH.load()};  }
std::pair<int,int> QtPlatformUIAdapter::GetWindowSize()      const { return {m_winW.load(), m_winH.load()}; }
bool QtPlatformUIAdapter::IsGPUAccelerated() const { return true; }
bool QtPlatformUIAdapter::ShouldCloseWindow() const { return m_shouldClose.load(); }

// ---------------- 把 mjr 切到 OFFSCREEN 模式 ----------------
bool QtPlatformUIAdapter::RefreshMjrContext(const mjModel* m, int fontscale) {
    int w = m_fbW.load(), h = m_fbH.load();
    {
        FILE* f = fopen("c:/Users/Administrator/Desktop/robotSim/qt-mujoco/demo/build/dbg.log","a");
        if (f) {
            fprintf(f, "RefreshMjrContext: m=%p fb=%dx%d before model.off=%dx%d\n",
                    (void*)m, w, h,
                    m ? m->vis.global.offwidth : -1,
                    m ? m->vis.global.offheight : -1);
            fclose(f);
        }
    }
    if (m && w > 0 && h > 0) {
        // mjModel 里 offwidth/offheight 是普通 int，可放心写入；这里只是把
        // 模型本身视为可变（simulate.cc 也在别处会修改 m->vis）。
        mjModel* mm = const_cast<mjModel*>(m);
        if (mm->vis.global.offwidth < w)  mm->vis.global.offwidth  = w;
        if (mm->vis.global.offheight < h) mm->vis.global.offheight = h;
    }

    bool changed = mujoco::PlatformUIAdapter::RefreshMjrContext(m, fontscale);
    {
        FILE* f = fopen("c:/Users/Administrator/Desktop/robotSim/qt-mujoco/demo/build/dbg.log","a");
        if (f) {
            fprintf(f, "RefreshMjrContext: changed=%d con.off=%dx%d offColor_r=%u offFBO=%u offFBO_r=%u\n",
                    int(changed), con_.offWidth, con_.offHeight, con_.offColor_r, con_.offFBO, con_.offFBO_r);
            fclose(f);
        }
    }
    if (changed) {
        // mjr_makeContext 默认 currentBuffer=mjFB_WINDOW；切到 OFFSCREEN，
        // 让 simulate.cc 内的所有 mjr_render / mjr_overlay / mjui_render
        // 都写入 con_.offFBO，无需修改官方 simulate 源码。
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);
        if (w > 0 && h > 0 && (con_.offWidth != w || con_.offHeight != h)) {
            mjr_resizeOffscreen(w, h, &con_);
        }
        m_offW.store(con_.offWidth);
        m_offH.store(con_.offHeight);
        EnsureSharedTarget(con_.offWidth, con_.offHeight);
    }
    return changed;
}

bool QtPlatformUIAdapter::EnsureContextSize() {
    int targetW = m_fbW.load(), targetH = m_fbH.load();
    if (targetW <= 0 || targetH <= 0) return false;
    if (con_.offWidth != targetW || con_.offHeight != targetH) {
        mjr_resizeOffscreen(targetW, targetH, &con_);
        // mjr_resizeOffscreen 不改 currentBuffer，但保险起见再保证一次
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);
        m_offW.store(con_.offWidth);
        m_offH.store(con_.offHeight);
        EnsureSharedTarget(con_.offWidth, con_.offHeight);
        return true;   // simulate.cc 会据此调用 UiModify 重新布局
    }
    return false;
}

// ---------------- 由 QtSimulateWindow / Quick item 间接做的 ----------------
void QtPlatformUIAdapter::SwapBuffers() {
    // simulate.cc::Render() 已把所有像素写入 con_.offFBO（多采样），
    // 这里把它解析到我们自己的可共享纹理上，供 Qt Quick 渲染线程采样。
    auto* glctx = QOpenGLContext::currentContext();
    if (glctx && con_.offWidth > 0 && con_.offHeight > 0 && con_.offFBO) {
        EnsureSharedTarget(con_.offWidth, con_.offHeight);
        unsigned int sharedFbo = m_sharedFbo;
        if (sharedFbo) {
            QOpenGLExtraFunctions* gl = glctx->extraFunctions();
            // 保存当前绑定，避免污染上层 GL 状态
            GLint prevReadFbo = 0, prevDrawFbo = 0;
            gl->glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prevReadFbo);
            gl->glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevDrawFbo);

            gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, con_.offFBO);
            gl->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, sharedFbo);
            gl->glBlitFramebuffer(0, 0, con_.offWidth, con_.offHeight,
                                  0, 0, con_.offWidth, con_.offHeight,
                                  GL_COLOR_BUFFER_BIT, GL_LINEAR);

            gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, prevReadFbo);
            gl->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, prevDrawFbo);

            // 强制把渲染指令完成，确保跨 context 共享纹理时另一边能立刻读到结果
            gl->glFlush();
        }
    }

    // 通知宿主：scenegraph 该重绘了
    if (m_host) m_host->onFrameRendered();
}

void QtPlatformUIAdapter::SetVSync(bool /*enabled*/) {
    // 渲染目标是 FBO，没有 swap 概念；交给 Quick scenegraph 控制刷新节奏
}

void QtPlatformUIAdapter::SetWindowTitle(const char* t) {
    if (m_host) m_host->onSetTitle(QString::fromUtf8(t));
}
void QtPlatformUIAdapter::ToggleFullscreen() {
    if (m_host) m_host->onToggleFullscreen();
}
void QtPlatformUIAdapter::SetClipboardString(const char* text) {
    QGuiApplication::clipboard()->setText(QString::fromUtf8(text));
}

mjtButton QtPlatformUIAdapter::TranslateMouseButton(int btn) const {
    switch (btn) {
        case 1: return mjBUTTON_LEFT;
        case 2: return mjBUTTON_RIGHT;
        case 3: return mjBUTTON_MIDDLE;
        default: return mjBUTTON_NONE;
    }
}

// ---------------- 事件队列 ----------------
void QtPlatformUIAdapter::PollEvents() {
    std::deque<std::function<void()>> drained;
    {
        std::lock_guard<std::mutex> lk(m_qMtx);
        drained.swap(m_queue);
    }
    for (auto &fn : drained) fn();
}

void QtPlatformUIAdapter::PostMouseButton(int qtBtn, int act, double x, double y) {
    {
        std::lock_guard<std::mutex> lk(m_cursorMtx);
        m_cursorX = x; m_cursorY = y;
    }
    bool down = (act == 1);
    if      (qtBtn == 1) m_btnLeft.store(down);
    else if (qtBtn == 2) m_btnRight.store(down);
    else if (qtBtn == 3) m_btnMiddle.store(down);

    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, qtBtn, act] { OnMouseButton(qtBtn, act); });
}

void QtPlatformUIAdapter::PostMouseMove(double x, double y) {
    {
        std::lock_guard<std::mutex> lk(m_cursorMtx);
        m_cursorX = x; m_cursorY = y;
    }
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, x, y] { OnMouseMove(x, y); });
}

void QtPlatformUIAdapter::PostScroll(double dx, double dy) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, dx, dy] { OnScroll(dx, dy); });
}

void QtPlatformUIAdapter::PostKey(int mjKey, int act) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    // TranslateKeyCode 是恒等映射，传过去的就是 mjKey
    m_queue.emplace_back([this, mjKey, act] { OnKey(mjKey, /*scancode*/0, act); });
}

void QtPlatformUIAdapter::PostResize(int w, int h) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, w, h] { OnWindowResize(w, h); });
}

void QtPlatformUIAdapter::PostClose() { m_shouldClose.store(true); }

void QtPlatformUIAdapter::SetModifiers(bool ctrl, bool shift, bool alt) {
    m_modCtrl.store(ctrl);
    m_modShift.store(shift);
    m_modAlt.store(alt);
}

void QtPlatformUIAdapter::SetWindowGeometry(int win_w, int win_h, int fb_w, int fb_h, double dpi) {
    m_winW.store(win_w);  m_winH.store(win_h);
    m_fbW.store(fb_w);    m_fbH.store(fb_h);
    m_dpi.store(dpi);
}

} // namespace mjqt
