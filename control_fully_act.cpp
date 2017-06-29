#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::fully_act_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    motors->set_update_rate(400);
    g.frame_type = AP_Motors::MOTOR_FRAME_TYPE_FULLY_ACT; //JL
    motors->init(AP_Motors::MOTOR_FRAME_HEXA, AP_Motors::MOTOR_FRAME_TYPE_FULLY_ACT); //JL

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::fully_act_run()
{
    //float target_forward, target_lateral;
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    //hal.console->printf("help meeeee!\n");
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    /*float alpha1 = 6;
    float alpha2 = 3;
    float d1 = 1;
    float d2 = 10;
    float xy_scale = 500.0f;
    float fx = channel_pitch->get_control_in()*xy_scale;
    float fy = channel_roll->get_control_in()*xy_scale;
    float fz = 0.0f;
    float taux = 0.0f;
    float tauy = 0.0f;
    float tauz = 0.0f;

    //Substitute scaled forces and torques to get omega here
    float om1 =  0.166666666666667f*fx + 0.288675134594813f*fy + 0.166666666666667f*fz - 0.166666666666667f*taux - 0.288675134594813f*tauy + 0.166666666666667f*tauz
    float om2 = -0.333333333333333f*fx +               0.0f*fy + 0.166666666666667f*fz - 0.333333333333333f*taux +               0.0f*tauy - 0.166666666666667f*tauz
    float om3 =  0.166666666666667f*fx - 0.288675134594813f*fy + 0.166666666666667f*fz - 0.166666666666667f*taux + 0.288675134594813f*tauy + 0.166666666666667f*tauz
    float om4 =  0.166666666666667f*fx + 0.288675134594813f*fy + 0.166666666666667f*fz + 0.166666666666667f*taux + 0.288675134594813f*tauy - 0.166666666666667f*tauz
    float om5 = -0.333333333333333f*fx +               0.0f*fy + 0.166666666666667f*fz + 0.333333333333333f*taux +               0.0f*tauy + 0.166666666666667f*tauz
    float om6 =  0.166666666666667f*fx - 0.288675134594813f*fy + 0.166666666666667f*fz + 0.166666666666667f*taux - 0.288675134594813f*tauy - 0.166666666666667f*tauz

*/
    motors->set_forward(channel_pitch->norm_input()*0.67f);
    motors->set_lateral(channel_roll->norm_input()*0.67f);


    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    // get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);      //JL
    get_pilot_desired_lean_angles(0.0f, 0.0f, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
