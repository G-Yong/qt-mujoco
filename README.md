# qt-mujoco

将 [MuJoCo](https://github.com/google-deepmind/mujoco) 官方 [`Simulate`](https://github.com/google-deepmind/mujoco/tree/main/simulate) 查看器以 QML 组件的形式嵌入 Qt Quick 应用的集成库。无需 GLFW，直接在 QML 场景中运行完整的 MuJoCo 物理仿真与交互式 3D 渲染。

## 效果

![snapshot](assets/snapshot.png)

## 特性

- **零 GLFW 依赖**：用 `QtPlatformUIAdapter` 替换官方 `GlfwAdapter`，所有 OpenGL 上下文、事件均由 Qt 管理。
- **QML 原生组件**：`MujocoView` 继承 `QQuickFramebufferObject`，可与任意 QML 布局、锚点、动画无缝组合。
- **跨上下文共享纹理**：MuJoCo 渲染线程持有私有 `QOffscreenSurface` + `QOpenGLContext`；渲染结果通过 `Qt::AA_ShareOpenGLContexts` 共享的 GL 纹理传递给 Qt Quick scenegraph，无像素级 CPU 回读。
- **帧节拍同步**：MuJoCo 渲染线程严格等待 Qt Quick scenegraph 取走上一帧后再进入下一次 `mjr_render`，避免以 GPU 极限速率生成被丢弃的帧，大窗口下交互保持流畅。
- **三线程架构**：渲染线程、物理仿真线程、Qt 主线程各司其职，互不阻塞。
- **拖拽加载模型**：直接将 `.xml` / `.mjb` 文件拖入窗口即可热切换模型。
- **独立 GPU 优先**：导出 `NvOptimusEnablement` / `AmdPowerXpressRequestHighPerformance` 符号，在双显卡笔记本上自动选用独立 GPU。

## 架构

```
Qt 主线程
└── MujocoView (QQuickFramebufferObject / QML 组件)
        │  鼠标 / 键盘 / 滚轮事件 → PostXxx() → 事件队列
        │
        ├── 渲染线程  (QOffscreenSurface + 私有 QOpenGLContext)
        │       └── mujoco::Simulate::RenderLoop()
        │               └── mjr_render → con_.offFBO (multisample)
        │               └── SwapBuffers: blit → 共享 GL 纹理 → glFlush
        │               └── 等待 scenegraph 消费信号（帧节拍）
        │
        ├── 物理线程
        │       └── mj_step / mj_forward 循环
        │
        └── Qt Quick scenegraph 渲染线程
                └── MujocoFboRenderer::render()
                        └── 将共享纹理 blit 到 Quick FBO → 发出消费信号
```

| 类 | 职责 |
|---|---|
| `MujocoQuickItem` | QML 可用的 `QQuickFramebufferObject`，管理生命周期、输入事件转发 |
| `MujocoFboRenderer` | scenegraph 渲染线程端：把共享纹理 blit 到 Quick 提供的 FBO |
| `QtPlatformUIAdapter` | 实现 `mujoco::PlatformUIAdapter`：offscreen FBO 管理、共享纹理创建、帧节拍 CV、事件队列 |

`MujocoQuickItem.h` 是可交付给外部客户的公共头文件。它只包含 Qt / C++ 标准库头和 `simulationtypes.h`，不包含 MuJoCo 或 `simulate` 目录下的头文件；需要 `mjModel` / `mjData` 的高级回调接口仅在头文件中前向声明类型。真正依赖 MuJoCo 的 `QtPlatformUIAdapter.h`、`simulate.h`、`mujoco.h` 只在实现文件或内部适配器头中使用。

在 `RobotSimulator` 动态库中集成时，构建系统通过 `MUJOCOQUICKITEM_EXPORT=Q_DECL_EXPORT` 导出 `MujocoQuickItem`。交付头 `simulationview.h` 会在包含 `MujocoQuickItem.h` 前将 `MUJOCOQUICKITEM_EXPORT` 映射为 `ROBOTSIMULATOR_EXPORT`，因此客户仍可使用兼容的 `RobotSim::SimulationView` 类型名；该类型名现在是 `MujocoQuickItem` 的别名，不再维护一层重复转发封装。

### 关键设计说明

| 问题 | 解决方案 |
|---|---|
| `con_.offColor_r` 是 renderbuffer，不能跨 context 共享也不能作为纹理采样 | 适配器自行创建 `GL_TEXTURE_2D` + 配套 FBO，在 `SwapBuffers` 里把 multisample offFBO blit 解析到该纹理 |
| `QOpenGLContext` / `QOffscreenSurface` 在渲染线程结束后线程亲和性失效 | 渲染线程退出前调用 `moveToThread(nullptr)` 交出所有权，主线程 `stop()` 再 `moveToThread(currentThread())` 后删除 |
| 大窗口下旋转 / 移动场景卡顿 | `condition_variable` 帧节拍：每帧渲染后等待 scenegraph 消费，使 mjr 循环速率自动与显示器刷新率对齐 |

## 依赖

| 组件 | 版本 |
|---|---|
| Qt | 5.15.2（需含 `quick`、`opengl` 模块）|
| MuJoCo | 3.8.0 Windows x86_64 |
| 编译器 | MSVC 2019 64-bit（`/utf-8`）|
| OpenGL | 3.3 Compatibility Profile |

## 快速开始

**1. 克隆并配置路径**

在 `demo/main.cpp` 中将 `initialXmlPath` 改为你自己的模型路径：

```cpp
engine.rootContext()->setContextProperty(
    "initialXmlPath",
    QStringLiteral("path/to/your/model.xml"));
```

**2. 用 Qt Creator 打开**

打开 `demo/demo.pro`，选择 `Desktop Qt 5.15.2 MSVC2019 64bit` Kit，直接构建运行。

**3. 双显卡笔记本**

`main.cpp` 顶部已导出 `NvOptimusEnablement` 和 `AmdPowerXpressRequestHighPerformance` 符号，NVIDIA / AMD 驱动会自动将本进程切至独立 GPU。

将此代码段复制到你自己项目的 `main.cpp` 顶部（必须在主可执行文件中，静态库 / DLL 中无效）：

```cpp
#if defined(_WIN32)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif
```

## 在自己的项目中使用

**C++ 端**：在 `.pro` 文件中引入，并在 `main()` 里注册 QML 类型：

```qmake
include(path/to/src/qt-mujoco.pri)
```

```cpp
// main.cpp
QGuiApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
QGuiApplication::setAttribute(Qt::AA_ShareOpenGLContexts); // 必须

qmlRegisterType<MujocoQuickItem>("Mujoco", 1, 0, "MujocoView");
```

**QML 端**：

```qml
import Mujoco 1.0

MujocoView {
    anchors.fill: parent
    focus: true

    Component.onCompleted: start("path/to/model.xml")
}
```

**C++ 热切换模型**（线程安全）：

```cpp
mujocoViewItem->loadScene("new_model.xml");
```

**通过 RobotSimulator 交付库使用**：

```cpp
#include "robotSimulator.h"

RobotSim::SimulationController controller;
RobotSim::SimulationView *view = controller.simulationView();

view->setSimulationRunning(false);
view->loadScene("robot.mjb");
```

`RobotSim::SimulationView` 只是兼容旧代码的类型别名，所有属性、信号和 `Q_INVOKABLE` 方法都来自 `MujocoQuickItem` 本体。`SimulationController` 创建的视图默认关闭左右 MuJoCo 内置 UI；直接实例化 `MujocoQuickItem` 时仍保留 qt-mujoco demo 的默认 UI 行为。

**QML 拖拽加载**：demo 中的 `DropArea` 示例可直接复用，将 `.xml` / `.mjb` 拖入窗口即可切换。

## 模型库

MuJoCo 官方提供了丰富的示例模型，可在 [MuJoCo 模型库](https://mujoco.readthedocs.io/en/stable/models.html) 中找到。

---

## 对 MuJoCo vendored 源码的补丁说明

> 升级 `mujoco-*-windows-x86_64` 时，需将以下改动重新应用到新版本对应文件。

### 背景

中上方的 `PAUSE / LOADING...` 覆盖文字由官方 `simulate.cc::Simulate::Render()` 直接调用
`mjr_overlay(mjFONT_BIG, mjGRID_TOP, ...)` 绘制进离屏 FBO，无法在封装层拦截。
为此给 `Simulate` 添加了 `status_overlay` 开关与 `status_overlay_text` 只读缓冲，
再通过 `MujocoQuickItem::statusOverlayVisible` / `statusOverlayText` 属性暴露给 QML。

[补丁请查看 `patches/status-overlay.patch`](patches/status-overlay.patch)，共 4 处改动，均有 `// qt-mujoco patch` 注释标明。

### 升级步骤

1. 将新版 `mujoco-X.Y.Z-windows-x86_64/` 目录放到本仓库同级目录，更新 `src/qt-mujoco.pri` 中的 `MUJOCO_DIR`。
2. 在新版 `simulate/` 目录下执行：
   ```bash
   git apply --directory=mujoco-X.Y.Z-windows-x86_64 patches/status-overlay.patch
   ```
   若补丁因上下文偏移无法自动应用，对照上方注释（`// qt-mujoco patch`）手动合并，改动点共 **4 处**：
   - `simulate.h`：在 `pause_update` 下方加 `status_overlay` 字段；在 `load_error` 下方加 `status_overlay_text` 字段。
   - `simulate.cc`：在 `zoom_increment` 之后加 `UpdateStatusOverlayText()` 函数；在 `Render()` 开头调用它；将两处绘制 overlay 的代码改为受 `status_overlay` 开关控制。