#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>//使用地面站时需要

void ModeNewmode::run()
{
    //gcs().send_text(MAV_SEVERITY_INFO, ">>> ENTER NEWMODE <<<");//调试用
    reset_controllers();
    state = GROUND_RUN;
    heading_set = false;
    heading_set= false;
   
}

void ModeNewmode::update()
{
    /*float ground_speed = plane.ahrs.groundspeed(); // 拿到地速 m/s
    float yaw = plane.ahrs.yaw_sensor;
    float takeoff_heading;
    static bool heading_set= false;

    switch (state) {
        case GROUND_RUN:
            if (!heading_set) {
                takeoff_heading = plane.ahrs.yaw_sensor;
                heading_set = true;
            } 
            // 油门100%
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);//SRV_Channels飞控的伺服通道管理核心类,飞控所有对外输出的设备（油门电调、舵机、电机等）都叫伺服通道.set_output_scaled不用关心硬件的具体 PWM 脉冲值，只需要传 0~100 的百分比，飞控自动转换成硬件能识别的信号。k_throttle 专门代表油门通道，枚举值，代表这个油门通道。
            // 保持水平
            plane.nav_pitch_cd = 0;
            float heading_error = takeoff_heading - plane.ahrs.yaw_sensor;
            
            // 处理角度环绕
            if (heading_error > 18000) heading_error -= 36000;
            if (heading_error < -18000) heading_error += 36000;
            // 简单比例控制（先用最简单版本）
            plane.nav_roll_cd = heading_error * 0.5f;

            if (plane.nav_roll_cd > 3000) plane.nav_roll_cd = 3000;
            if (plane.nav_roll_cd < -3000) plane.nav_roll_cd = -3000;
                
            static bool rotate_sent = false;

            // 到速度 → 抬头
            if (ground_speed > 12.0f) {   // 这个速度你要调

                state = ROTATE;

                if (!rotate_sent) {
                gcs().send_text(MAV_SEVERITY_INFO, "ROTATE!");
                rotate_sent = true;
                }
                    
            }
        break;
        
        case ROTATE:
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
            // 抬头 12°
            plane.nav_pitch_cd = 1600;
            // 保持不滚转
            plane.nav_roll_cd = 0;
            break;
        }
        */
    float ground_speed = plane.ahrs.groundspeed();
    float yaw = plane.ahrs.yaw_sensor;
     static bool rotate_sent = false;

    switch (state) {
        case GROUND_RUN:{
            if (!heading_set&& ground_speed > 3.0f) {
                takeoff_heading = yaw;
                heading_set = true;
            }//航向锁定

            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);//直接拉满油门

            plane.nav_pitch_cd = 0;
            plane.nav_roll_cd = 0;

            float heading_error = takeoff_heading - yaw;

            if (heading_error > 18000) heading_error -= 36000;
            if (heading_error < -18000) heading_error += 36000;

            float rudder = heading_error * 0.5f;

            if (rudder > 4500) rudder = 4500;
            if (rudder < -4500) rudder = -4500;

            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);

            if (ground_speed > 12.0f) {
                state = ROTATE;
                if (!rotate_sent) {
                gcs().send_text(MAV_SEVERITY_INFO, "ROTATE!");
                rotate_sent = true;
                } 
            }
        }
        break;

        case ROTATE:{

            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);//这个是最底层的控制，直接输出给舵机。
            plane.nav_pitch_cd = 800;  // 改小
          
            float heading_error = takeoff_heading - yaw;
            if (heading_error > 18000) heading_error -= 36000;
            if (heading_error < -18000) heading_error += 36000;
            
            
            float roll_cmd = heading_error * 0.1f;
            roll_cmd -= plane.ahrs.roll_sensor * 0.4f;

          
            if (roll_cmd > 800) roll_cmd = 800;
            if (roll_cmd < -800) roll_cmd = -800;

            plane.nav_roll_cd = roll_cmd;

        }
        break;
    }
}

bool ModeNewmode::use_throttle_limits() const
{
    return false;
}
