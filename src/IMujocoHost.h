#pragma once

#include <QString>

namespace mjqt {

class IMujocoHost {
public:
    virtual ~IMujocoHost() = default;
    virtual void onFrameRendered() = 0;
    virtual void onSetTitle(const QString& title) = 0;
    virtual void onToggleFullscreen() = 0;
};

} // namespace mjqt