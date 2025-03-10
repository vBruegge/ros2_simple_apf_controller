int32 CIRCULAR = 0
int32 RECTANGULAR = 1

geometry_msgs/Point origin
int32 shape

float32 length #if CIRCULAR: diameter
float32 width #if CIRCULAR: diameter
