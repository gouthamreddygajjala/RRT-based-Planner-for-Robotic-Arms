string name
uint8 type
float64[] params        # [radius] / [radius, length] / [lx, ly, lz] etc.
geometry_msgs/Pose pose

# Primitive type constants
uint8 SPHERE   = 0
uint8 CYLINDER = 1
uint8 BOX      = 2
uint8 CAPSULE  = 3
uint8 CONE     = 4
