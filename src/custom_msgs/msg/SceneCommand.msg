uint8 action

PrimitiveObject primitive   # used when action = ADD_PRIMITIVE
MeshObject mesh             # used when action = ADD_MESH
string remove_name          # used when action = REMOVE


# Action constants
uint8 ADD_PRIMITIVE = 0
uint8 ADD_MESH      = 1
uint8 REMOVE        = 2
uint8 CLEAR         = 3