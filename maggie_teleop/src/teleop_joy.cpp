/**
 * @file        teleop_joy.cpp
 * @brief       Node for controlling a joystick for teleoperation.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-04
 * @author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
 * @date        2012-04
 *
 * @copyright   Copyright (C) 2015 University Carlos III of Madrid.
 *              All rights reserved.
 * @license     LEUC3M v1.0, see LICENSE.txt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the Licencia Educativa UC3M as published by
 * the University Carlos III of Madrid, either version 1.0, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. See the Licencia Educativa UC3M
 * version 1.0 or any later version for more details.
 *
 * A copy of the Licencia Educativa UC3M is in the LICENSE file.
 */

#include "teleop_joy.h"

//////////////////////////////////////////////////

TeleopJoy::TeleopJoy()
{
    _nh.param("axis_analog_linear", axis_analog_linear, 3);
    _nh.param("axis_analog_angular", axis_analog_angular, 2);
    _nh.param("axis_digital_linear", axis_digital_linear, 1);
    _nh.param("axis_digital_angular", axis_digital_angular, 0);
    _nh.param("scale_angular", scale_angular, (double) 1);
    _nh.param("scale_linear", scale_linear, (double) 1);

    _vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // This topics must be checked to match the subscribers (if already existing)

    _right_arm_vel_pub = _nh.advertise<geometry_msgs::Twist>("body/right_arm_cmd_vel", 1);
    _left_arm_vel_pub = _nh.advertise<geometry_msgs::Twist>("body/left_arm_cmd_vel", 1);

    _neck_pan_vel_pub = _nh.advertise<geometry_msgs::Twist>("body/neck_pan_cmd_vel", 1);
    _neck_tilt_vel_pub = _nh.advertise<geometry_msgs::Twist>("body/neck_tilt_cmd_vel", 1);

    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joy_callback, this);
}

//////////////////////////////////////////////////

TeleopJoy::~TeleopJoy()
{
}

//////////////////////////////////////////////////

void TeleopJoy::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Drawing for the code of the buttons on the Logitech RumblePad 2:
    //   ---------------
    //  |  [6]     [7]  |
    //  |  [4]     [5]  |
    //   ---------------
    //  |   |      (3)  |
    //  | --+--  (0) (2)|
    //  |   |      (1)  |
    //  / /-----------\ \
    // / /

    geometry_msgs::Twist vel;
    vel.linear.x = scale_linear * (joy->axes[axis_analog_linear] + joy->axes[axis_digital_linear]);
    vel.angular.z = scale_angular * (joy->axes[axis_analog_angular] + joy->axes[axis_digital_angular]);

    // Buttons 0, 1, 2, 3 corresponds to 1, 2, 3, 4 in the joypad
    // Button 0 + joystick = move right arm
    // Button 1 + joystick = move left arm
    // Button 2 + joystick = pan head
    // Button 3 + joystick = tilt head

    if (joy->buttons[0]) {
        ROS_DEBUG_THROTTLE(1, "Button 0 pushed. Moving right arm. \n Emitting linear: %g, angular: %g", vel.linear.x,
                          vel.angular.z);

        _right_arm_vel_pub.publish(vel);
    }
    else if (joy->buttons[1]) {
        ROS_DEBUG_THROTTLE(1, "Button 1 pushed. Moving left arm. \n Emitting linear: %g, angular: %g", vel.linear.x,
                          vel.angular.z);

        _left_arm_vel_pub.publish(vel);
    }
    else if (joy->buttons[2]) {
        ROS_DEBUG_THROTTLE(1, "Button 2 pushed. Panning head. \n Emitting linear: %g, angular: %g", vel.linear.x,
                          vel.angular.z);

        _neck_pan_vel_pub.publish(vel);
    }
    else if (joy->buttons[3]) {
        ROS_DEBUG_THROTTLE(1, "Button 3 pushed. Tilting head. \n Emitting linear: %g, angular: %g", vel.linear.x,
                          vel.angular.z);

        _neck_tilt_vel_pub.publish(vel);
    }
    else {
        ROS_DEBUG_THROTTLE(1, "Emitting linear: %g, angular: %g", vel.linear.x, vel.angular.z);

        _vel_pub.publish(vel);
    }
}

//////////////////////////////////////////////////
