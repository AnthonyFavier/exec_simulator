uint8 type
# TYPES:
uint8 PASSIVE   =   0   
uint8 PICK_OBJ  =   1   # obj
uint8 PLACE_OBJ =   2   # obj, location
uint8 PICK_B    =   3   
uint8 PICK_R    =   4   
uint8 PICK_G    =   5   
uint8 PICK_Y    =   6   
uint8 PICK_W    =   7   
uint8 PICK_P    =   8   
uint8 PLACE_1   =   9   # obj
uint8 PLACE_2   =   10  # obj
uint8 PLACE_3   =   11  # obj
uint8 PLACE_4   =   12  # obj
uint8 PLACE_5   =   13  # obj
uint8 PUSH      =   14  
uint8 OPEN_BOX  =   15  
uint8 DROP      =   16  


# PARAMS:
geometry_msgs/Pose pose
string named_target
string location
string obj
