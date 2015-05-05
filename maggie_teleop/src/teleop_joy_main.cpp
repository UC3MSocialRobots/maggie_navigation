/**
 * @file        teleop_joy_main.h
 * @brief       Main for controlling a joystick for teleoperation.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-04
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_joy");

    TeleopJoy teleop_joy;

    ros::spin();
}
