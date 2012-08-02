# A 3D map in binary format, as Octree
Header header
# The pose of the octree with respect to the header frame 
geometry_msgs/Pose origin
# binary serialization of octree, use conversions.h to read and write octrees
int8[] data
