uint8 MALE = 1
uint8 FEMALE = 2

string name
float32 name_score  # the higher, the more probable 'name' is correct

uint32 age
float32 age_score

uint8 gender
float32 gender_score

geometry_msgs/PoseStamped pose
string body_pose
