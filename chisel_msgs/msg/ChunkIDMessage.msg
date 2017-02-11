#Chunks spatial position
int32 ID_x
int32 ID_y
int32 ID_z

#Chunk world position
geometry_msgs/Vector3 origin

#optional fields for voxel IDs of tracked voxel changes
int32[] carved_voxels
int32[] updated_voxels
