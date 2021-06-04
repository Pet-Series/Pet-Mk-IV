|CMakeLists.txt|
| ------------- |
```txt
cmake_minimum_required(VERSION 2.8.3)

project(tianbot_mini_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
```

| package.xml  |
| ------------- |
```xml
<package format="2">
  <name>tianbot_mini_description</name>
  <version>1.0.0</version>
  <description>
    <p>URDF Description package for tianbot_mini_description</p>
    <p>This package contains configuration data, 3D models and launch files
for tianbot_mini_description robot</p>
  </description>
  <author>TODO</author>
  <maintainer email="sunMaxwell@outlook.com">fancheWang</maintainer>
  <license>GPLv3</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roslaunch</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz</depend>
  <depend>joint_state_publisher</depend>
  <depend>gazebo</depend>
  <export>
    <architecture_independent />
  </export>
</package>
```
