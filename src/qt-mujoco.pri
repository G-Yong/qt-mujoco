HEADERS += \
    $$PWD/IMujocoHost.h \
    $$PWD/QtPlatformUIAdapter.h \
    $$PWD/MujocoQuickItem.h \
    $$PWD/simulationtypes.h \
    $$PWD/lodepng.h

SOURCES += \
    $$PWD/QtPlatformUIAdapter.cpp \
    $$PWD/MujocoQuickItem.cpp

# Quick / QML 集成需要 quick 模块
QT *= core gui qml quick


MUJOCO_DIR = $$PWD/../mujoco-3.8.0-windows-x86_64
message("Using MuJoCo at: $$MUJOCO_DIR")

INCLUDEPATH += $$MUJOCO_DIR/include
# 官方 simulate 源码所在目录（有 simulate.h / platform_ui_adapter.h）
INCLUDEPATH += $$MUJOCO_DIR/simulate
# 让 simulate.cc 找到我们提供的 lodepng 替身
INCLUDEPATH += $$PWD

SOURCES += \
    $$MUJOCO_DIR/simulate/simulate.cc \
    $$MUJOCO_DIR/simulate/platform_ui_adapter.cc

win32 {
    LIBS += -L$$MUJOCO_DIR/lib -lmujoco
    LIBS += -lopengl32

    LIBS += -L$$MUJOCO_DIR/bin

    # main.cpp 中已通过导出 NvOptimusEnablement / AmdPowerXpressRequestHighPerformance
    # 强制 NVIDIA Optimus / AMD PowerXpress 混合显卡选用独立 GPU，
    # 系统硬件 OpenGL 驱动可正常提供 ARB_framebuffer_object，无需额外操作。
}
unix:!macx {
    LIBS += -L$$MUJOCO_DIR/lib -lmujoco -lGL
    QMAKE_RPATHDIR += $$MUJOCO_DIR/lib
}