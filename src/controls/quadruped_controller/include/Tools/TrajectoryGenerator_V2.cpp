#include "TrajectoryGenerator_V2.h"

TrajectoryGenerator::TrajectoryGenerator() : kEpsilon(0.0000001), kPi(3.14159265358979323846)
{
    Initialize();
}
TrajectoryGenerator::~TrajectoryGenerator()
{
    // TODO Auto-generated destructor stub
}
void TrajectoryGenerator::Initialize()
{
    trajectory_type_ = SINUSOIDAL; // default type
    state_ = 0;
    state_old_ = 0;
    dt_ = 0.001; // default: 1000hz -> 0.001s
    t_ = 0;
    gt_ = 0;
    ellip_via_=0;
    trajectory_mode_ = POINT_SINGLE;
    pause_ = false;
    has_final_position = false;
    has_final_velocity = false;
    has_final_acceleration = false;
    has_bezier_middle_point = false;
    has_duration = false;
    is_computed_once = false;
    is_trajectory_init_=false;
    is_more_than_half_=false;
    lead_axis_=X;
    for (size_t i = 0; i < NUM_AXIS; i++)
    {
        pi_[i] = 0;
        pf_[i] = 0;
        vi_[i] = 0;
        vf_[i] = 0;
        ai_[i] = 0;
        af_[i] = 0;
        bezier_init_[i] = false;
    }
}
void TrajectoryGenerator::ComputeTrajectory()
{
    if (is_computed_once == false)
    {
        if (has_final_position == false)
        {
            std::cout << "You must set a final position" << std::endl;
            return;
        }
        if (has_duration == false)
        {
            std::cout << "You must set a duratinon" << std::endl;
            return;
        }
        if (has_final_velocity == false)
        {
            for (size_t axis = 0; axis < trajectory_mode_; axis++)
            {
                vf_[axis] = 0;
                th_dot_f_[axis] = 0;
            }
            has_final_velocity = true;
        }
        if (has_final_acceleration == false)
        {
            for (size_t axis = 0; axis < trajectory_mode_; axis++)
            {
                af_[axis] = 0;
                th_ddot_f_[axis] = 0;
            }
            has_final_acceleration = true;
        }

        if (!(tf_ < kEpsilon))
            inv_tf_ = 1 / tf_;
        else
            inv_tf_ = 0;
        end_time_ = tf_ + waiting_time_;
        if (is_trajectory_init_ == false)
        {
            t_ = 0;
            is_trajectory_init_=true;
        }
        else
            t_ = dt_;
        state_old_ = state_;
        state_ = -1;
        for (size_t axis = 0; axis < trajectory_mode_; axis++)
        {
            bezier_init_[axis] = false;
            if(is_initialize_point == false&&state_old_!=-1) //if (state_old_ != 0 && is_initialize_point == false)
            {
                FinalValueToInitialValue(axis);
            }

            if (trajectory_type_ != QUINTIC&&trajectory_type_ != BEZIER&& trajectory_type_ != ELLIPSOID && (fabs(vi_[axis]) > kEpsilon || fabs(ai_[axis]) > kEpsilon || fabs(vf_[axis]) > kEpsilon || fabs(af_[axis]) > kEpsilon))
            {
                trajectory_type_ = QUINTIC;
                quintic_coefficient_[axis][0] = pi_[axis];
                quintic_coefficient_[axis][1] = vi_[axis];
                quintic_coefficient_[axis][2] = ai_[axis] / 2.;
                quintic_coefficient_[axis][3] = (20 * pf_[axis] - 20 * pi_[axis] + tf_ * (af_[axis] * tf_ - 3 * ai_[axis] * tf_ - 8 * vf_[axis] - 12 * vi_[axis])) / (2. * pow(tf_, 3));
                quintic_coefficient_[axis][4] = (-30 * pf_[axis] + 30 * pi_[axis] + tf_ * (-2 * af_[axis] * tf_ + 3 * ai_[axis] * tf_ + 14 * vf_[axis] + 16 * vi_[axis])) / (2. * pow(tf_, 4));
                quintic_coefficient_[axis][5] = (12 * pf_[axis] - 12 * pi_[axis] + tf_ * ((af_[axis] - ai_[axis]) * tf_ - 6 * (vf_[axis] + vi_[axis]))) / (2. * pow(tf_, 5));

                std::cout << "Trajectory type is changed to Quintic" << std::endl;
            }
            else if (trajectory_type_ == QUINTIC)
            {
                quintic_coefficient_[axis][0] = pi_[axis];
                quintic_coefficient_[axis][1] = vi_[axis];
                quintic_coefficient_[axis][2] = ai_[axis] / 2.;
                quintic_coefficient_[axis][3] = (20 * pf_[axis] - 20 * pi_[axis] + tf_ * (af_[axis] * tf_ - 3 * ai_[axis] * tf_ - 8 * vf_[axis] - 12 * vi_[axis])) / (2. * pow(tf_, 3));
                quintic_coefficient_[axis][4] = (-30 * pf_[axis] + 30 * pi_[axis] + tf_ * (-2 * af_[axis] * tf_ + 3 * ai_[axis] * tf_ + 14 * vf_[axis] + 16 * vi_[axis])) / (2. * pow(tf_, 4));
                quintic_coefficient_[axis][5] = (12 * pf_[axis] - 12 * pi_[axis] + tf_ * ((af_[axis] - ai_[axis]) * tf_ - 6 * (vf_[axis] + vi_[axis]))) / (2. * pow(tf_, 5));
            }
            else if (trajectory_type_ == LSPB)
            {
                v_const_[axis] = fabs(v_const_[axis]) * Sgn(pf_[axis] - pi_[axis]);
                //Too small or Too big velocity
                if (fabs(v_const_[axis]) < (fabs(pf_[axis] - pi_[axis]) * inv_tf_) || fabs(v_const_[axis]) > (2 * fabs(pf_[axis] - pi_[axis]) * inv_tf_))
                {
                    v_const_[axis] = 1.5 * (pf_[axis] - pi_[axis]) * inv_tf_;
                    std::cout << "LSPB constant Velocity is changed" << std::endl;
                }
                if (fabs(v_const_[axis]) > kEpsilon)
                    tb_[axis] = (pi_[axis] - pf_[axis] + v_const_[axis] * tf_) / v_const_[axis];
                else
                    tb_[axis] = 0.;
                if (fabs(tb_[axis]) > kEpsilon)
                    a_const_[axis] = v_const_[axis] / tb_[axis];
                else
                    a_const_[axis] = 0.;
            }
            else if (trajectory_type_== ELLIPSOID)
            {
                SetQuinticCoe(pi_[X],pf_[X],0,0,0,0,tf_,X);
                SetQuinticCoe(pi_[Y],pf_[Y],0,0,0,0,tf_,Y);
                SetQuinticCoe(pi_[Z],pi_[Z]+semi_axis_z_,0,0,0,0,tf_*0.5,Z);
                ellip_via_ = 1;
                
                if (fabs(pf_[X]-pi_[X])>kEpsilon)
                {
                    lead_axis_=X;
                }
                else if (fabs(pf_[Y] - pi_[Y]) > kEpsilon)
                {
                    lead_axis_ = Y;
                }
                else
                {
                    lead_axis_=Z;                 
                    return;
                }

                if (lead_axis_==X||lead_axis_==Y)
                {
                    semi_axis_x_ = 0.5 * fabs(pf_[lead_axis_] - pi_[lead_axis_]);
                    // semi_axis_z_ = pf_[Z] - pi_[Z];
                    ellip_center_x_ = pi_[lead_axis_] + 0.5 * (pf_[lead_axis_] - pi_[lead_axis_]);
                    ellip_center_z_ = pi_[Z];
                }
                
            }
        }
        is_computed_once = true;
    }

    for (size_t axis = 0; axis < trajectory_mode_; axis++)
    {
        switch (trajectory_type_)
        {
        case SINUSOIDAL:
            if (t_ < tf_ - kEpsilon) // t_<=tf_
            {
                p_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * (1 - std::cos(kPi * t_ * inv_tf_)) + pi_[axis];
                v_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * std::sin(kPi * t_ * inv_tf_) * kPi * inv_tf_;
                a_[axis] = (pf_[axis] - pi_[axis]) * 0.5 * std::cos(kPi * t_ * inv_tf_) * kPi * kPi * inv_tf_ * inv_tf_;
            }
            else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
            break;

        case LSPB:
            if (t_ < (tb_[axis] - kEpsilon))
            {
                p_[axis] = pi_[axis] + a_const_[axis] * 0.5 * t_ * t_;
                v_[axis] = a_const_[axis] * t_;
                a_[axis] = a_const_[axis];
            }
            else if (t_ < (tf_ - tb_[axis] - kEpsilon))
            {
                p_[axis] = (pf_[axis] + pi_[axis] - v_const_[axis] * tf_) * 0.5 + v_const_[axis] * t_;
                v_[axis] = v_const_[axis];
                a_[axis] = 0.;
            }
            else if (t_ < (tf_ - kEpsilon))
            {
                p_[axis] = pf_[axis] - a_const_[axis] * 0.5 * tf_ * tf_ + a_const_[axis] * tf_ * t_ - a_const_[axis] * 0.5 * t_ * t_;
                v_[axis] = a_const_[axis] * tf_ - a_const_[axis] * t_;
                a_[axis] = -a_const_[axis];
            }

            else if (t_ < end_time_ + kEpsilon)
            {
                p_[axis] = pf_[axis];
                v_[axis] = 0.;
                a_[axis] = 0.;
            }
            break;

        case QUINTIC:
            if (t_ < tf_ - kEpsilon) //same as t_<=tf_
            {
                p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
            }
            else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
            break;

        case BEZIER:
        {
            if (trajectory_mode_ == POINT_XYZ)
            {
                if (t_ < tf_ * 0.5)
                {
                    is_more_than_half_ = false;
                }
                else
                {
                    is_more_than_half_ = true;
                }

                double t_temp = tf_ * 0.3;
                if (t_ < t_temp - kEpsilon)
                {
                    if (bezier_init_[axis] == false)
                    {
                        ComputeTrajectoryUsingTime(t_temp, axis); //Compute p_,v_,a_ at t=t_temp
                        SetQuinticCoe(pi_[axis], p_[axis], vi_[axis], v_[axis], ai_[axis], a_[axis], t_temp, axis);
                        bezier_init_[axis] = true;
                    }
                    //Quintic trajectory

                    p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                    v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                    a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
                }
                else if (t_ < tf_  - kEpsilon) //same as t_<=tf_  //tf_ - t_temp - kEpsilon
                {
                    //Quadratic
                    // p_[axis] = pm_[axis] + (1 - t_ * inv_tf_) * (1 - t_ * inv_tf_) * (pi_[axis] - pm_[axis]) + t_ * inv_tf_ * t_ * inv_tf_ * (pf_[axis] - pm_[axis]);
                    // v_[axis] = 2 * (1 - t_ * inv_tf_) * (pm_[axis] - pi_[axis]) + 2 * t_ * inv_tf_ * (pf_[axis] - pm_[axis]);
                    // a_[axis] = 2 * (pf_[axis] - 2 * pm_[axis] + pi_[axis]);
                    //Cubic
                    p_[axis] = pow(1 - t_ * inv_tf_,3)* pi_[axis] + 3*pow(1 - t_ * inv_tf_,2)*t_ * inv_tf_* pm_[axis]+3*(1 - t_ * inv_tf_)*pow(t_ * inv_tf_,2)* pm2_[axis]+pow(t_ * inv_tf_,3)* pf_[axis];
                    v_[axis] = 3 * pow(1 - t_ * inv_tf_,2) * (pm_[axis] - pi_[axis]) + 6 *(1 - t_ * inv_tf_)* t_ * inv_tf_ * (pm2_[axis] - pm_[axis])+3 * pow(t_ * inv_tf_,2) * (pf_[axis] - pm2_[axis]);
                    a_[axis] = 6*(1 - t_ * inv_tf_) * (pm2_[axis] - 2 * pm_[axis] + pi_[axis]) + 6 * t_ * inv_tf_*(pf_[axis] - 2 * pm2_[axis] + pm_[axis]);

                }
                else if (t_ < end_time_ + kEpsilon) //duration < t < end_time
                {
                    p_[axis] = pf_[axis];
                    v_[axis] = 3 * (pf_[axis] - pm2_[axis]);
                    a_[axis] = 6 * (pf_[axis] - 2 * pm2_[axis] + pm_[axis]);
                }
            }
            break;
        }
        case ELLIPSOID:
        {
            double a = semi_axis_x_;
            double b = semi_axis_z_;
            double x = p_[lead_axis_];
            double x_dot = v_[lead_axis_];
            double x_ddot = a_[lead_axis_];
            double x0 = ellip_center_x_;
            double z0 = ellip_center_z_;
            if (ellip_via_ == 1 && t_ >= tf_ * 0.5-kEpsilon && lead_axis_ == Z)
            {
                SetQuinticCoe(pi_[Z]+semi_axis_z_, pi_[Z], 0, 0, 0, 0, tf_* 0.5 , Z);
                ellip_via_ = 0;
            }
            if (t_ < tf_ * 0.5)
            {
                is_more_than_half_ = false;
            }
            else
            {
                is_more_than_half_=true;
            }

            if (t_ < tf_ - kEpsilon  && axis!=Z) //same as t_<=tf_ 
            {
                p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
            }
            else if (t_ < end_time_ + kEpsilon && axis!=Z) //duration < t < end_time  
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
             
            if (t_ < tf_ - kEpsilon && lead_axis_ != Z && axis == Z)
            {
                p_[axis] = b * sqrt(1 + kEpsilon - pow(x - x0, 2) / (a * a)) + z0;
                v_[axis] = (-(x - x0) * x_dot / (a * a)) * b * b / (p_[Z] - z0);
                a_[axis] = ((-(x_dot * x_dot + (x - x0) * x_ddot) / (a * a)) * b * b - v_[Z] * v_[Z]) / (p_[Z] - z0);
            }
            else if (t_ < tf_*0.5 - kEpsilon &&lead_axis_ == Z && axis == Z) // 
            {
                p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * t_ + quintic_coefficient_[axis][2] * t_ * t_ + quintic_coefficient_[axis][3] * t_ * t_ * t_ + quintic_coefficient_[axis][4] * t_ * t_ * t_ * t_ + quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_ * t_;
                v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * t_ + 3 * quintic_coefficient_[axis][3] * t_ * t_ + 4 * quintic_coefficient_[axis][4] * t_ * t_ * t_ + 5 * quintic_coefficient_[axis][5] * t_ * t_ * t_ * t_;
                a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * t_ + 12 * quintic_coefficient_[axis][4] * t_ * t_ + 20 * quintic_coefficient_[axis][5] * t_ * t_ * t_;
            }
             else if (t_ >=tf_*0.5 - kEpsilon  &&lead_axis_ == Z && axis == Z) // lead_axis_ == Z &&
            {
                p_[axis] = quintic_coefficient_[axis][0] + quintic_coefficient_[axis][1] * (t_ - tf_ * 0.5) + quintic_coefficient_[axis][2] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + quintic_coefficient_[axis][3] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + quintic_coefficient_[axis][4] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + quintic_coefficient_[axis][5] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5);
                v_[axis] = quintic_coefficient_[axis][1] + 2 * quintic_coefficient_[axis][2] * (t_ - tf_ * 0.5) + 3 * quintic_coefficient_[axis][3] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + 4 * quintic_coefficient_[axis][4] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + 5 * quintic_coefficient_[axis][5] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5);
                a_[axis] = 2 * quintic_coefficient_[axis][2] + 6 * quintic_coefficient_[axis][3] * (t_ - tf_ * 0.5) + 12 * quintic_coefficient_[axis][4] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) + 20 * quintic_coefficient_[axis][5] * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5) * (t_ - tf_ * 0.5);
            }
            else if (t_ < end_time_ + kEpsilon && axis == Z) //duration < t < end_time
            {
                p_[axis] = pf_[axis];
                v_[axis] = vf_[axis];
                a_[axis] = af_[axis];
            }
            break;
        }
        default:
            std::cout << axis << " Trajectory Unknown\n";
            break;
        }
    }
    IncreaseTimeAndStateUpdate();
}
void TrajectoryGenerator::FinalValueToInitialValue(int axis)
{
    pi_[axis] = p_[axis];
    vi_[axis] = v_[axis];
    ai_[axis] = a_[axis];
}
void TrajectoryGenerator::SetQuinticCoe(const double pi, const double pf, const double vi, const double vf, const double ai, const double af, const double tf, size_t axis)
{
    quintic_coefficient_[axis][0] = pi;
    quintic_coefficient_[axis][1] = vi;
    quintic_coefficient_[axis][2] = ai / 2.;
    quintic_coefficient_[axis][3] = (20 * pf - 20 * pi + tf * (af * tf - 3 * ai * tf - 8 * vf - 12 * vi)) / (2. * pow(tf, 3));
    quintic_coefficient_[axis][4] = (-30 * pf + 30 * pi + tf * (-2 * af * tf + 3 * ai * tf + 14 * vf + 16 * vi)) / (2. * pow(tf, 4));
    quintic_coefficient_[axis][5] = (12 * pf - 12 * pi + tf * ((af - ai) * tf - 6 * (vf + vi))) / (2. * pow(tf, 5));
}
void TrajectoryGenerator::ComputeTrajectoryUsingTime(double time, size_t axis)
{
    switch (trajectory_type_)
    {
    case BEZIER:
    {
        if (time < tf_ - kEpsilon) //same as time<=tf_
        {
            // p_[axis] = pm_[axis] + (1 - time * inv_tf_) * (1 - time * inv_tf_) * (pi_[axis] - pm_[axis]) + time * inv_tf_ * time * inv_tf_ * (pf_[axis] - pm_[axis]);
            // v_[axis] = 2 * (1 - time * inv_tf_) * (pm_[axis] - pi_[axis]) + 2 * time * inv_tf_ * (pf_[axis] - pm_[axis]);
            // a_[axis] = 2 * (pf_[axis] - 2 * pm_[axis] + pi_[axis]);
            p_[axis] = pow(1 - time * inv_tf_, 3) * pi_[axis] + 3 * pow(1 - time * inv_tf_, 2) * time * inv_tf_ * pm_[axis] + 3 * (1 - time * inv_tf_) * pow(time * inv_tf_, 2) * pm2_[axis] + pow(time * inv_tf_, 3) * pf_[axis];
            v_[axis] = 3 * pow(1 - time * inv_tf_, 2) * (pm_[axis] - pi_[axis]) + 6 * (1 - time * inv_tf_) * time * inv_tf_ * (pm2_[axis] - pm_[axis]) + 3 * pow(time * inv_tf_, 2) * (pf_[axis] - pm2_[axis]);
            a_[axis] = 6 * (1 - time * inv_tf_) * (pm2_[axis] - 2 * pm_[axis] + pi_[axis]) + 6 * time * inv_tf_ * (pf_[axis] - 2 * pm2_[axis] + pm_[axis]);
        }

        else if (time < end_time_ + kEpsilon) //duration < time < end_time
        {
            p_[axis] = pf_[axis];
            v_[axis] = 0.;
            a_[axis] = 0.;
        }

        break;
    }
    default:
        std::cout << "Trajectory Unknown\n";
        break;
    }
}
void TrajectoryGenerator::IncreaseTimeAndStateUpdate()
{
    if (t_ < end_time_ - dt_ + kEpsilon && pause_ == false)
    {
        t_ += dt_;
    }
    else if (state_ == -1)
    {
        state_ = state_old_;
        state_++;
        // state_old_ = state_;
        has_final_position = false;
        has_final_velocity = false;
        has_final_acceleration = false;
        has_bezier_middle_point = false;
        has_duration = false;
        is_initialize_point = false;
    }
}
void TrajectoryGenerator::BreakingUp()
{
    t_ = end_time_;
    IncreaseTimeAndStateUpdate();
}
void TrajectoryGenerator::SetEllipsoid(int ellip_axis,double ellip_height,bool positive)
{
    semi_axis_z_=ellip_height;
}