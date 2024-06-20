int32 id
uint8 type
# TYPES:
uint8 PASSIVE           =   0   
uint8 PICK_OBJ          =   1   # color, side
uint8 PLACE_OBJ         =   2   # location
uint8 PUSH              =   3  
uint8 OPEN_BOX          =   4  
uint8 DROP              =   5  
uint8 PICK_OBJ_NAME     =   6   # obj_name
uint8 PLACE_OBJ_NAME    =   7   # obj_name, location
uint8 TURN_AROUND       =   8
uint8 MOVE_FORWARD      =   9
uint8 ASK               =   10  # obj_name, location


# PARAMS:
geometry_msgs/Pose pose
string named_target
string location
string color
string side
string obj_name
