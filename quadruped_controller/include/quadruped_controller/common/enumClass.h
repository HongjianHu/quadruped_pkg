#ifndef ENUMCLASS_H
#define ENUMCLASS_H

namespace quadruped_controller
{

enum class FSMStateName
{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDDOWN,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,

    SWINGTEST,
    BALANCETEST,
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

enum class FrameType
{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus
{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

} // namespace quadruped_controller

#endif // ENUMCLASS_H