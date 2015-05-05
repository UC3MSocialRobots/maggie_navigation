#ifndef __TELEOP_JOY_H__
#define __TELEOP_JOY_H__

/**
 * @file        teleop_joy.h
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

#include <ros/ros.h>

// messages
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class TeleopJoy {
    public:
        /**
         * @brief Empty constructor.
         */
        TeleopJoy();

        /**
         * @brief Destructor.
         */
        ~TeleopJoy();

    private:
        /**
         * @brief
         * @param joy The joy message.
         * @return
         */
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

        // node
        ros::NodeHandle _nh;

        int axis_analog_linear, axis_analog_angular;
        int axis_digital_linear, axis_digital_angular;
        double scale_linear, scale_angular;

        // base publisher
        ros::Publisher _vel_pub;

        // arms publishers
        ros::Publisher _right_arm_vel_pub;
        ros::Publisher _left_arm_vel_pub;

        // neck publishers
        ros::Publisher _neck_pan_vel_pub;   // movement around the z axis
        ros::Publisher _neck_tilt_vel_pub;  // movement around the y axis

        // joy subscriber
        ros::Subscriber _joy_sub;
};

#endif
