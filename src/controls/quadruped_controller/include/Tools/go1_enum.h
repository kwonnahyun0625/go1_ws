#ifndef GO1_ENUM_H_
#define GO1_ENUM_H_

enum Hertz
{
    HZ_CONTROL = 1000, 
};
enum Joint
{
    HR,
    HP,
    KP,
    NUM_JOINT,
};
enum Gait
{
    WALK,
    TROT,
};
enum Leg_Num
{
    F1,
    F2,
    H1,
    H2,
    NUM_LEG,
};
enum Leg_Num2
{
    LF,
    RF,
    LR,
    RR,
};
enum DoF
{
    NUM_DOF = NUM_JOINT * NUM_LEG,
};
enum Pos
{
    X,
    Y,
    Z,
    NUM_AXIS,
};
enum VelCmd
{
    CMD_X_VEL,
    CMD_Y_VEL,
    CMD_YAW_VEL,
    NUM_VEL_CMD = 3,
};
enum RPY
{
    ROLL,
    PITCH,
    YAW,
    NUM_RPY,
};
enum TrajectoryType
{
    SINUSOIDAL = 0,
    LSPB = 1,
    QUINTIC = 2,
    BEZIER = 3,
    ELLIPSOID = 4,
    ELLIPSOID2 = 5,
};
enum TrajectoryMode
{
    SINGLE_AXIS = 0,
    POINT_SINGLE = 1,
    POINT_XYZ = 3,
};
enum ControlMode
{
    INIT,
    HOMING,
    TASKSPACE,
    WBC_TEST,
    RUNNING,
    RUNNING_WM,

    NUM_MODE,
}; 
enum BodyState
{
    ST_ROLL,
    ST_PITCH,
    ST_YAW,
    ST_X,
    ST_Y,
    ST_Z,
    ST_ROLL_DOT,
    ST_PITCH_DOT,
    ST_YAW_DOT,
    ST_X_DOT,
    ST_Y_DOT,
    ST_Z_DOT,
    ST_G,
    ST_NUM = 13,
}; 
#endif
 