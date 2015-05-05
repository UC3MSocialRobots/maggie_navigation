# How To Map with Maggie

Launch files are inside 'maggie_navigation_config/launch'

Launch Maggie mapping:
   
```shell
$ roslaunch maggie_navigation_config maggie_mapping.launch
```

Once the map is ready, save it:
   
```shell
$ rosrun map_server map_saver -f "map_name"
```




