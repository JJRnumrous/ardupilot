#include "Copter.h"
#include "SITL/SIM_Gazebo.h"
/*
 * required to write to file
 */
#include <iostream>
#include <fstream>
using namespace std;

/*
 * Init and run calls for vision flight mode
 */
bool commanded;
bool command;


bool Copter::ModeVision::init(bool ignore_checks)
{
    ofstream myfile;
    myfile.open ("../../Build/example.txt", ios::out);
//
//    ofstream cntrfile;
//    cntrfile.open ("../../Build/control.txt", ios::out);
//    cntrfile.close();
//
//    ofstream cntfile;
//    cntfile.open ("../../Build/control_vel.txt", ios::out);
//    cntfile.close();


    commanded = false;
    command = false;

    return true;
}


void Copter::ModeVision::run()
{
    ofstream myfile;
    myfile.open ("../../Build/example.txt", ios::out | ios::app);

    double time_now = time_stmp;

    myfile << time_now;
    myfile << "\t";
//    printf("%ld", time_now);
//             printf("\n");


    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    float target = 0.0f;
//     accel target = -700 | -690 | -590 | 0


    if(time_now<30.0){

        target = 0.0f;      // 0 m/s*s
    }else if(time_now < 35.0){
        if(!commanded){
            command = true;
            commanded = true;
        }
        target = 1000.0f;
    }else if(time_now < 40.0){

        target = 1000.0f;    // 1 m/s*s
    }else
    {

        target = 1000.0f;    // 1 m/s*s
    }

    myfile << radians(target*0.1);
//    myfile << target*0.05;
    myfile << "\t";

//    float current = run_roll_angle_controller(target);
//    float current = run_roll_rate_controller(target);
//    float current = run_pitch_angle_controller(target);
//    float current = run_pitch_rate_controller(target);
//    float current = run_yaw_angle_controller(target);
    float current = run_yaw_rate_controller(target);
//    float current = run_alt_controller(target);
//    float current = run_climb_rate_cnt(target*0.1);

//    float current = run_nsa_controller(target*0.05);

    //run_motors(time_now);
    //run_test_motors();


    myfile << current;
    myfile << "\n";



}

float Copter::ModeVision::run_roll_angle_controller(float target)
{

    if (command)
    {
        attitude_control->input_angle_step_bf_roll_pitch_yaw(target,0.0f,0.0f);
        command = false;
    }else{
        attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
    }
    run_alt_controller(1000.0f);

    return ahrs.get_rotation_body_to_ned().to_euler312().x;
}



float Copter::ModeVision::run_roll_rate_controller(float target)
{

       attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
       attitude_control->rate_bf_yaw_target(0.0f);
       attitude_control->rate_bf_roll_target(target);
       attitude_control->rate_bf_pitch_target(0.0f);
       run_alt_controller(1000);
       Vector3f temp = ahrs.get_gyro_latest();
       return temp.x;
}


float Copter::ModeVision::run_pitch_angle_controller(float target)
{

    if (command)
    {
        attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,target,0.0f);
        command = false;
    }else{
        attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
    }
    run_alt_controller(1000.0f);

    return ahrs.get_rotation_body_to_ned().to_euler312().y;
}


float Copter::ModeVision::run_pitch_rate_controller(float target)
{

       attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
       attitude_control->rate_bf_yaw_target(0.0f);
       attitude_control->rate_bf_roll_target(0.0f);
       attitude_control->rate_bf_pitch_target(target);
       run_alt_controller(1000);
       Vector3f temp = ahrs.get_gyro_latest();
       return temp.y;
}



float Copter::ModeVision::run_yaw_angle_controller(float target)
{

    if (command)
    {
        attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,target);
        command = false;
    }else{
        attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
    }
    run_alt_controller(1000.0f);

    return ahrs.get_rotation_body_to_ned().to_euler312().z;
}


/*
 * yaw rate controller
 * sets all other angles zero and altitude to 10m = 1 000cm
 */
float Copter::ModeVision::run_yaw_rate_controller(float target)
{
    attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f,0.0f,0.0f);
    attitude_control->rate_bf_yaw_target(target);
    attitude_control->rate_bf_roll_target(0.0f);
    attitude_control->rate_bf_pitch_target(0.0f);
    run_alt_controller(1000);
    Vector3f temp = ahrs.get_gyro_latest();
    return temp.z;
}

/*
 * Altitude Controller
 * testing the z position controller
 */
float Copter::ModeVision::run_alt_controller(float target)
{
    pos_control->set_alt_target(target);
    pos_control->set_accel_z(250);
    return pos_control->update_z_controller();
}

/*
 * Climbrate controller
 * testing the z velocity gains
 */
float Copter::ModeVision::run_climb_rate_cnt(float target)
{
    return pos_control->run_z_vel_controller(target);
}

/* *****************************************************************************
     *  NSA Controller
     *  for testing the accelration gains
    **************************************************************************** */
float Copter::ModeVision::run_nsa_controller(float target)
{

        pos_control->run_z_accel_controller(target);
        float current = -(ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
        return current;
}

/*
  *  Test motor output
  *  receive rc in and write directly to motors
  */
void Copter::ModeVision::run_test_motors()
{

    for(uint8_t i = 0 ; i < 14 ; i++) {
        uint16_t pwm = hal.rcin->read(i);
        hal.rcout->write(i, pwm + i);
    }
}

/*
  *  Test motor
  *  by setting values directly
  */
void Copter::ModeVision::run_motors(uint32_t time_now)
{
      motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
      motors->set_roll(0.0);
      motors->set_pitch(0.0);
      motors->set_yaw(0.0);

      if(time_now <= 30000)
      {
          attitude_control->set_throttle_out(0.3, false, 50);
          motors->set_throttle(0.3);
      }else if (time_now <= 25000)
      {
          //motors->set_throttle(0.6);
          attitude_control->set_throttle_out(0.3, false, 50);
      }else if (time_now <= 30000)
      {
          //motors->set_throttle(0.6);
          attitude_control->set_throttle_out(0.4, false, 50);
      }else if (time_now <= 35000)
      {
          //motors->set_throttle(0.6);
          attitude_control->set_throttle_out(0.5, false, 50);
      }else
      {
          //motors->set_throttle(0.6);
          attitude_control->set_throttle_out(0.6, false, 50);
      }
}

