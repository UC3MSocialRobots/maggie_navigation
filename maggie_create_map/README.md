# maggie_create_map

ROS package for creating a map with the Social Robot Maggie.

## How To Map with Maggie

Launch files are inside `maggie_navigation_config/launch`

Launch Maggie mapping:

```shell
$ roslaunch maggie_navigation_config maggie_mapping.launch robot:=robot_name
```

Once the map is ready, save it:

```shell
$ rosrun map_server map_saver -f "map_name" __ns:=/robot_name
```

## LICENSE

The license of the packages is custom LASR-UC3M (Licencia Académica Social Robotics Lab - UC3M), an open, non-commercial license which enables you to download, modify and distribute the code as long as you distribute the sources.

## ACKNOWLEDGEMENTS

![RoboticsLab](http://ieee.uc3m.es/images/thumb/b/b6/Roboticslab_text_new.jpg/128px-Roboticslab_text_new.jpg)
![UC3M](http://ieee.uc3m.es/images/thumb/6/6b/Logo_uc3m_letras.png/256px-Logo_uc3m_letras.png)
