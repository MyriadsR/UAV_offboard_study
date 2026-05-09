/*
 * @file attitude_offboard_minimal_test.cpp
 * @brief Minimal PX4 MAVROS Offboard attitude + thrust test
 *
 * 功能：
 *   1. 等待 FCU 连接
 *   2. 持续发布水平姿态 + 固定推力 setpoint
 *   3. 切换 OFFBOARD
 *   4. 解锁
 *   5. 继续保持水平姿态 + 固定推力
 *
 * 注意：
 *   这是最小接口测试，不做位置闭环，不做高度闭环，不自动降落。
 *   如果 thrust 偏大，飞机会持续上升；如果 thrust 偏小，飞机不会离地。
 */

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

mavros_msgs::State current_state;

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

static double clamp01(double x)
{
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

static double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

static geometry_msgs::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    return tf2::toMsg(q);
}

static mavros_msgs::AttitudeTarget makeAttitudeTarget(
    double roll_rad,
    double pitch_rad,
    double yaw_rad,
    double thrust)
{
    mavros_msgs::AttitudeTarget target;

    target.header.stamp = ros::Time::now();

    // 使用 orientation + thrust，忽略机体系角速度 body_rate。
    // 不要设置 IGNORE_ATTITUDE，也不要设置 IGNORE_THRUST。
    target.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    target.orientation = rpyToQuaternion(roll_rad, pitch_rad, yaw_rad);

    target.body_rate.x = 0.0;
    target.body_rate.y = 0.0;
    target.body_rate.z = 0.0;

    // MAVROS/PX4 这里是归一化推力，通常范围 [0, 1]，不是牛顿。
    target.thrust = clamp01(thrust);

    return target;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_offboard_minimal_test");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // -----------------------------
    // 可调参数
    // -----------------------------
    double rate_hz;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
    double thrust;

    pnh.param("rate_hz", rate_hz, 50.0);
    pnh.param("roll_deg", roll_deg, 0.0);
    pnh.param("pitch_deg", pitch_deg, 0.0);
    pnh.param("yaw_deg", yaw_deg, 0.0);

    // 最小测试建议先用 0.55 ~ 0.65。
    // 如果不起飞，逐步加大；如果太猛，逐步减小。
    pnh.param("thrust", thrust, 0.7);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, stateCallback);

    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    ros::Rate rate(rate_hz);

    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected.");

    const double roll_rad = deg2rad(roll_deg);
    const double pitch_rad = deg2rad(pitch_deg);
    const double yaw_rad = deg2rad(yaw_deg);

    ROS_INFO("Minimal attitude test command: roll = %.2f deg, pitch = %.2f deg, yaw = %.2f deg, thrust = %.3f",
             roll_deg, pitch_deg, yaw_deg, thrust);

    mavros_msgs::AttitudeTarget target = makeAttitudeTarget(
        roll_rad, pitch_rad, yaw_rad, thrust);

    // -----------------------------
    // 进入 OFFBOARD 前必须先发送一段 setpoint
    // -----------------------------
    ROS_INFO("Sending pre-offboard setpoints...");
    for (int i = 0; ros::ok() && i < 100; ++i)
    {
        target.header.stamp = ros::Time::now();
        attitude_pub.publish(target);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_print = ros::Time::now();

    ROS_INFO("Entering main loop. Press Ctrl+C to stop.");

    while (ros::ok())
    {
        // 每个循环都持续发布 setpoint，不能中断。
        target.header.stamp = ros::Time::now();
        attitude_pub.publish(target);

        // 尝试切换 OFFBOARD。
        if (current_state.mode != "OFFBOARD" &&
            ros::Time::now() - last_request > ros::Duration(2.0))
        {
            if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
            {
                ROS_INFO("OFFBOARD enabled.");
            }
            else
            {
                ROS_WARN("Failed to enable OFFBOARD. Current mode: %s", current_state.mode.c_str());
            }
            last_request = ros::Time::now();
        }
        // OFFBOARD 之后尝试解锁。
        else if (current_state.mode == "OFFBOARD" &&
                 !current_state.armed &&
                 ros::Time::now() - last_request > ros::Duration(2.0))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed.");
            }
            else
            {
                ROS_WARN("Failed to arm vehicle.");
            }
            last_request = ros::Time::now();
        }

        // 低频打印当前状态，方便排查。
        if (ros::Time::now() - last_print > ros::Duration(1.0))
        {
            ROS_INFO("state: connected=%d, armed=%d, mode=%s, thrust=%.3f",
                     current_state.connected,
                     current_state.armed,
                     current_state.mode.c_str(),
                     thrust);
            last_print = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
