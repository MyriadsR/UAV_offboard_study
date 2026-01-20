/*
 * @Author: xzr && error: git config user.email & please set dead value or install git
 * @Date: 2026-01-19 16:12:12
 * @LastEditors: xzr && error: git config user.email & please set dead value or install git
 * @LastEditTime: 2026-01-19 18:20:14
 * @FilePath: /offboard_test_ws/src/offboard/src/offboard_node.cpp
 * @Description: 
 * 
 * Copyright (c) 2026 by error: git config user.email & please set dead value or install git, All Rights Reserved. 
 */
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::ExtendedState current_extended_state;
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    current_extended_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
            ("mavros/extended_state", 10, extended_state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    ros::Time hover_start_time;
    ros::Time land_start_time;
    bool hover_started = false;
    bool landing_started = false;
    bool disarm_sent = false;
    
    enum FlightState {
        TAKEOFF,
        MOVING,
        HOVERING,
        LANDING,
        LANDED
    };
    FlightState flight_state = TAKEOFF;
    
    ros::Time move_start_time;
    bool move_started = false;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 状态机控制飞行流程
        switch(flight_state) {
            case TAKEOFF:
                // 检查是否到达起始位置（容差0.2m）
                if(current_state.armed && current_state.mode == "OFFBOARD") {
                    double distance = sqrt(
                        pow(current_pose.pose.position.x - pose.pose.position.x, 2) +
                        pow(current_pose.pose.position.y - pose.pose.position.y, 2) +
                        pow(current_pose.pose.position.z - pose.pose.position.z, 2)
                    );
                    if(distance < 0.2) {
                        flight_state = MOVING;
                        ROS_INFO("Reached takeoff position (0, 0, 1.5m), moving forward 1m");
                    }
                }
                local_pos_pub.publish(pose);
                break;
                
            case MOVING: {
                // 向x正方向移动1m
                if(!move_started) {
                    move_started = true;
                    move_start_time = ros::Time::now();
                    ROS_INFO("Starting movement to (1, 0, 1.5m)");
                }
                
                // 以0.5m/s的速度向x方向移动（2秒移动1m）
                double move_time = (ros::Time::now() - move_start_time).toSec();
                double target_x = 0.5 * move_time;
                if(target_x > 1.0) target_x = 1.0;
                
                pose.pose.position.x = target_x;
                pose.pose.position.y = 0;
                pose.pose.position.z = 1.5;
                local_pos_pub.publish(pose);
                
                // 检查是否到达目标位置
                double dist_to_target = sqrt(
                    pow(current_pose.pose.position.x - 1.0, 2) +
                    pow(current_pose.pose.position.y - 0.0, 2) +
                    pow(current_pose.pose.position.z - 1.5, 2)
                );
                
                if(dist_to_target < 0.2) {
                    hover_start_time = ros::Time::now();
                    hover_started = true;
                    flight_state = HOVERING;
                    ROS_INFO("Reached target position (1, 0, 1.5m), hovering for 2 seconds");
                }
                break;
            }
                
            case HOVERING:
                // 悬停2秒
                local_pos_pub.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(2.0)) {
                    flight_state = LANDING;
                    ROS_INFO("Hovering complete, initiating landing");
                }
                break;
                
            case LANDING: {
                // 在OFFBOARD模式下缓慢降低高度
                if(!landing_started) {
                    landing_started = true;
                    land_start_time = ros::Time::now();
                    ROS_INFO("Starting landing descent from position (1, 0, 1.5m)");
                }
                
                // 以0.3m/s的速度降低高度，保持x=1, y=0
                double descent_time = (ros::Time::now() - land_start_time).toSec();
                double target_z = 1.5 - 0.3 * descent_time;
                
                pose.pose.position.x = 1.0;
                pose.pose.position.y = 0.0;
                
                // 限制最低高度，留一点余量让飞控检测接地
                if(target_z < -0.1) target_z = -0.1;  // 目标稍微低于地面帮助飞控检测降落
                
                pose.pose.position.z = target_z;
                local_pos_pub.publish(pose);
                
                // 当实际高度很低且经过足够时间，认为已降落
                // 或者飞控已经报告降落状态
                bool altitude_low = (current_pose.pose.position.z < 0.2);
                bool time_enough = (descent_time > 6.0);  // 至少6秒确保完全降落
                bool fc_detected_landing = (current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND);
                
                if((altitude_low && time_enough) || fc_detected_landing) {
                    flight_state = LANDED;
                    ROS_INFO("Landed - altitude: %.2fm, time: %.1fs, fc_state: %d", 
                             current_pose.pose.position.z, descent_time, 
                             current_extended_state.landed_state);
                }
                break;
            }
                
            case LANDED: {
                // 停止发送OFFBOARD设定点，让飞控自己检测降落并上锁
                static bool mode_switched = false;
                
                if(!mode_switched) {
                    ROS_INFO("Landing complete - switching to MANUAL mode");
                    ROS_INFO("Flight controller will detect landing and disarm automatically");
                    
                    // 切换到MANUAL模式，让飞控自己处理降落检测和上锁
                    mavros_msgs::SetMode manual_mode;
                    manual_mode.request.custom_mode = "MANUAL";
                    
                    if(set_mode_client.call(manual_mode) && manual_mode.response.mode_sent) {
                        ROS_INFO("Switched to MANUAL mode - waiting for auto-disarm");
                        mode_switched = true;
                    } else {
                        ROS_WARN("Failed to switch mode - will retry");
                    }
                }
                
                // 切换模式后，等待飞控自动上锁
                if(mode_switched) {
                    static ros::Time switch_time;
                    static bool switch_time_set = false;
                    
                    if(!switch_time_set) {
                        switch_time = ros::Time::now();
                        switch_time_set = true;
                    }
                    
                    double wait_time = (ros::Time::now() - switch_time).toSec();
                    
                    // 检查是否已经上锁
                    if(!current_state.armed) {
                        ROS_INFO("Vehicle disarmed successfully by flight controller!");
                        ROS_INFO("Mission completed! Total time: %.1fs", wait_time);
                        disarm_sent = true;
                        
                        // 等待1秒后退出程序
                        ros::Duration(1.0).sleep();
                        ros::shutdown();
                    } else if(wait_time > 10.0) {
                        // 如果10秒后仍未自动上锁，退出程序让failsafe处理
                        ROS_INFO("Auto-disarm timeout - exiting to trigger failsafe");
                        ROS_INFO("Flight controller will handle landing and disarm");
                        ros::shutdown();
                    } else {
                        // 每2秒打印一次状态
                        static double last_info_time = 0;
                        if(wait_time - last_info_time > 2.0) {
                            ROS_INFO("Waiting for auto-disarm... (%.1fs, armed=%d, landed_state=%d)", 
                                     wait_time, current_state.armed, current_extended_state.landed_state);
                            last_info_time = wait_time;
                        }
                    }
                }
                
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}