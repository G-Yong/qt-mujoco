#-------------------------------------------------
#  demo.pro
#  Qt 5.15.2 + MuJoCo 3.8.0 集成演示 (qmake)
#-------------------------------------------------

QT       += core gui qml quick
CONFIG   += c++17
TARGET    = demo
TEMPLATE  = app

# MSVC 默认以 GBK 读取源文件，遇到 UTF-8 中文会报 C2001（常量中有换行符）
# /utf-8 = /source-charset:utf-8 /execution-charset:utf-8，一次性解决
win32-msvc* {
    QMAKE_CXXFLAGS += /utf-8
}

include(../src/qt-mujoco.pri)

# ----------------------------------------------------------------------------
# 源文件 / 资源
# ----------------------------------------------------------------------------
SOURCES += \
    main.cpp

RESOURCES += qml.qrc