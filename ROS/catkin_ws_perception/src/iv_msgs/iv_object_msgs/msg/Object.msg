std_msgs/Header header

int32 id # track id or detection sequence id

# existence confidence
float32 confidence

# object classification type, Car, Pedestrian, etc.
string type
uint8 type_id
# type probability
float32 type_prob

### Object properties
#object bounding box center [cx,cy,cz]
geometry_msgs/Vector3 center
# object bounding box size information [length, width, height]
geometry_msgs/Vector3 size
# object yaw angle [rad]
float32 orientation

### velocity information
# composite linear speed sqrt(x^2 + y^2 + z^2)
float32 speed
# velocity vector
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 absolute_velocity


# polygon points
geometry_msgs/Polygon polygon

# cloud points [optional]
geometry_msgs/Polygon cloud_points