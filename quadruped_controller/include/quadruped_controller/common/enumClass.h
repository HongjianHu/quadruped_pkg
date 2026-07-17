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
    MPC_TROTTING,
    MPC_WBC_TROTTING,

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

} // namespace quadruped_controller

#endif // ENUMCLASS_H
