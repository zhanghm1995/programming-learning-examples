# ROS-learning-examples
The `ROS` folder contains ROS related learning example codes, including basic **ROS** usages (in `catkin_ws_basic`) and some other useful codes using ROS.

## catkin_ws_basic

### Build

```bash
cd catkin_ws_basic
catkin_make -j8
source devel setup.bash
rosrun <package_name> <node_name>
```

If you want to compile specific ROS package, you can use:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"
```



### Contents

#### learning_rviz

Learning how to publish `visualization_msgs/Marker` to **rviz** and display it. 

For `visualization_msgs/Marker`, its [details](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html) contains:

```bash
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id                           # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type                         # Type of object
int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
```

For `ARROW` type, its `scale.x` determines the length of the `ARROW`.

#### Trouble shooting

- Nothing in Rviz

  You should check the`color.a`,  `lifetime` and `scale` value, especially the `color.a`, it should not be the default value 0. 