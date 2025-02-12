#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED == ENABLED // 别忘了这块在config.h和mode.cpp的定义

/*
 * Init and run calls for guided flight mode 五角星航线初始化
 */

// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    path_num = 0; // 航点号清零，从而切到其他模式再切回来后，可以飞出一个新的五角星航线
    generate_path(); // 生成五角星航线
    
    // start in wp control mode
    wp_control_start(); // 开始航点控制


    return true;
}

void ModeDrawStar::generate_path() // 生成五角星航线
{
    //float radius_cm = 1000.0;
    float radius_cm = g2.star_radius_cm; // 在Parameters中新增的变量

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];

}

// initialise guided mode's waypoint navigation controller
void ModeDrawStar::wp_control_start() //这个旧版本用的是pos_control_start()函数，但这个新版本不是
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();


    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if (path_num < 6){ // 五角星航点尚未走完
        if (wp_nav->reached_wp_destination()){  // 到达某个航点
            path_num ++;
            wp_nav->set_wp_destination(path[path_num], false); // 将下一个航点设置为导航控制模块的目标位置
        } 
     } else if ((path_num == 6) && wp_nav->reached_wp_destination()) {  // 五角星航线运行完成，自动进入Loiter模式
         gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
         copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    }


    wp_control_run();
}


// run guided mode's waypoint navigation controller
void ModeDrawStar::wp_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw 这一块与4.0.7版本不太相同，这里一条代码带过，我也没有更改，照常用了
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

#endif
