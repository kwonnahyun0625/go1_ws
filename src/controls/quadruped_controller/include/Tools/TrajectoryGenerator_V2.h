#ifndef TRAJEC_H_
#define TRAJEC_H_

#include <cmath>
#include <iostream>
#include "go1_enum.h"
class TrajectoryGenerator
{
private:
    
    const double kEpsilon;
    const double kPi;
    double dt_; //delta time, default 0.001s
    double t_;  // trajectory time -> 0~duration
    double gt_; // global time -> 0~ Inf  !!Unimplemented!!
    unsigned int trajectory_type_;
    unsigned int trajectory_mode_;
    double pi_[NUM_AXIS];      //initial position
    double pf_[NUM_AXIS];      //final position
    double vi_[NUM_AXIS];      //initial velocity
    double vf_[NUM_AXIS];      //final velocity
    double ai_[NUM_AXIS];      //initial acceleration
    double af_[NUM_AXIS];      //final acceleration
    double th_i_[NUM_AXIS];
    double th_dot_i_[NUM_AXIS];
    double th_ddot_i_[NUM_AXIS];
    double th_f_[NUM_AXIS];
    double th_dot_f_[NUM_AXIS];
    double th_ddot_f_[NUM_AXIS];
    double tf_;                //final time(=duration)
    double v_const_[NUM_AXIS]; //LSPB constant velocity
    double tb_[NUM_AXIS];      //LSPB trajectory blend time
    double a_const_[NUM_AXIS]; //LSPB constant acceleration
    double inv_tf_;            // =1/tf_
    double waiting_time_;
    double end_time_;
    double pm_[NUM_AXIS]; // for bezier trajectory(middle params)
    double pm2_[NUM_AXIS];
    int state_;           //arrived(0,1,2...) or moving(-1)
    int state_old_;
    double quintic_coefficient_[NUM_AXIS][6];

    //time-varying
    double p_[NUM_AXIS]; // position
    double v_[NUM_AXIS]; // velocity
    double a_[NUM_AXIS]; // acceleration
    double th_[NUM_AXIS];
    double th_dot_[NUM_AXIS];
    double th_ddot_[NUM_AXIS];

    //boolean
    bool pause_;
    bool bezier_init_[NUM_AXIS];
    bool has_final_position;
    bool has_final_velocity;
    bool has_final_acceleration;
    bool has_bezier_middle_point;
    bool has_duration;
    bool is_computed_once;
    bool is_initialize_point;
    bool is_trajectory_init_;

    //ellipsoidal
    bool is_more_than_half_;
    int lead_axis_;
    double semi_axis_x_;
    double semi_axis_y_;
    double semi_axis_z_;
    double ellip_center_x_;
    double ellip_center_y_;
    double ellip_center_z_;
    int ellip_via_;

public:
    TrajectoryGenerator();
    virtual ~TrajectoryGenerator();
    void SetTimeInterval(double dt) { dt_ = dt; }
    void SetTrajectoryType(int type) { trajectory_type_ = type; }
    void SetInitialPosition(double initial_position)
    {
        pi_[SINGLE_AXIS] = initial_position;
        is_initialize_point = true;
    }
    void SetInitialPosition(double initial_x, double initial_y, double initial_z)
    {
        pi_[X] = initial_x;
        pi_[Y] = initial_y;
        pi_[Z] = initial_z;
        vi_[X] = 0.;
        vi_[Y] = 0.;
        vi_[Z] = 0.;
        ai_[X] = 0.;
        ai_[Y] = 0.;
        ai_[Z] = 0.;
        trajectory_mode_ = POINT_XYZ;
        is_initialize_point = true;
    }
    void SetDt(double dt){dt_=dt;}
    void SetBasicParameters(double final_position, double duration_t, double wait_t = 0)
    {
        pf_[SINGLE_AXIS] = final_position;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBasicParameters(double final_x, double final_y, double final_z, double duration_t, double wait_t = 0)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalPosition(double final_position)
    {
        pf_[SINGLE_AXIS] = final_position;
        has_final_position = true;
        is_computed_once = false;
    }
    void SetFinalPosition(double final_x, double final_y, double final_z)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        has_final_position = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalVelocity(double final_velocity)
    {
        vf_[SINGLE_AXIS] = final_velocity;
        has_final_velocity = true;
    }
    void SetFinalVelocity(double final_x_dot, double final_y_dot, double final_z_dot)
    {
        vf_[X] = final_x_dot;
        vf_[Y] = final_y_dot;
        vf_[Z] = final_z_dot;
        has_final_velocity = true;
    }
    void SetFinalAcceleration(double final_acceleration)
    {
        af_[SINGLE_AXIS] = final_acceleration;
        has_final_acceleration = true;
    }
    void SetFinalAcceleration(double final_x_ddot, double final_y_ddot, double final_z_ddot)
    {
        af_[X] = final_x_ddot;
        af_[Y] = final_y_ddot;
        af_[Z] = final_z_ddot;
        has_final_acceleration = true;
    }

    void SetConstantVelocity(double const_velocity)
    {
        v_const_[SINGLE_AXIS] = const_velocity;
    }
    void SetConstantVelocity(double const_x_dot, double const_y_dot, double const_z_dot)
    {
        v_const_[X] = const_x_dot;
        v_const_[Y] = const_y_dot;
        v_const_[Z] = const_z_dot;
    }
    void SetDuration(double duration_t, double wait_t = 0)
    {
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBezierMiddlePosition(double bezier_middle_position)
    {
        pm_[SINGLE_AXIS] = bezier_middle_position;
        has_bezier_middle_point = true;
    }
    void SetBezierMiddlePosition(double bezier_mid_x, double bezier_mid_y, double bezier_mid_z)
    {
        pm_[X] = bezier_mid_x;
        pm_[Y] = bezier_mid_y;
        pm_[Z] = bezier_mid_z;
        has_bezier_middle_point = true;
    }
    void SetBezier2MiddlePosition(double bezier_mid_x, double bezier_mid_y, double bezier_mid_z)
    {
        pm2_[X] = bezier_mid_x;
        pm2_[Y] = bezier_mid_y;
        pm2_[Z] = bezier_mid_z;
        has_bezier_middle_point = true;
    }
    void SetEllipsoid(int ellip_axis,double ellip_height,bool positive=true);
    void SetEllipsoidParameters(double final_x, double final_y, double height_z,double final_th_x, double final_th_y,double duration_t, double wait_t = 0)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = pi_[Z];
        th_f_[X] = final_th_x;
        th_f_[Y] = final_th_y;
        semi_axis_x_=0.5*fabs(pf_[X]-pi_[X]);
        semi_axis_y_=0.5*fabs(pf_[Y]-pi_[Y]);
        semi_axis_z_=fabs(height_z-pi_[X]);
        ellip_center_x_=pi_[X]+0.5*(pf_[X]-pi_[X]);
        ellip_center_y_=pi_[Y]+0.5*(pf_[Y]-pi_[Y]);
        ellip_center_z_=pi_[Z];
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetEllipsoidFinalVelocity(double final_th_x_dot, double final_th_y_dot, double final_th_z_dot)
    {
        th_dot_f_[X] = final_th_x_dot;
        th_dot_f_[Y] = final_th_y_dot;
        th_dot_f_[Z] = final_th_z_dot;
        has_final_velocity = true;
    }
    void SetEllipsoidFinalAcceleration(double final_th_x_ddot, double final_th_y_ddot, double final_th_z_ddot)
    {
        th_ddot_f_[X] = final_th_x_ddot;
        th_ddot_f_[Y] = final_th_y_ddot;
        th_ddot_f_[Z] = final_th_z_ddot;
        has_final_acceleration = true;
    }

    void ComputeTrajectory();

    int GetState() { return state_; }
    int GetStateOld() { return state_old_; }
    double GetInitialPosition(int axis = SINGLE_AXIS) { return pi_[axis]; }
    double GetPosition(int axis = SINGLE_AXIS) { return p_[axis]; }
    double GetVelocity(int axis = SINGLE_AXIS) { return v_[axis]; }
    double GetAcceleration(int axis = SINGLE_AXIS) { return a_[axis]; }
    double *GetXYZ() { return p_; }
    double *GetXYZDot() { return v_; }
    double *GetXYZDdot() { return a_; }
    double GetFinalPosition(int axis = SINGLE_AXIS) { return pf_[axis]; }
    double GetTime(){return t_;}
    bool IsMoreThanHalf(){return is_more_than_half_;}
    void Resume() { pause_ = false; }
    void Pause() { pause_ = true; }
    void Reset() { state_ = 0; }
    void example();
    void BreakingUp();

private:
    void Initialize();
    void IncreaseTimeAndStateUpdate();
    void FinalValueToInitialValue(int axis = SINGLE_AXIS);
    int Sgn(double val) { return (kEpsilon < val) - (val < -kEpsilon); }
    void SetQuinticCoe(const double pi, const double pf, const double vi, const double vf, const double ai, const double af, const double tf, size_t axis);
    void ComputeTrajectoryUsingTime(double time, size_t axis);
};
#endif 
