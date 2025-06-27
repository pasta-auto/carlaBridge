int16 attack_type 0
float32 attack_angle 0
float32 attack_angle_dropoff 0.0349066
float32 distance_dropoff 0
float32 distance_less 0
float32 distance_greater 0
float32 z_min 0.1
float32 z_tolerance 0.01
float32 random_drop_percent 0.95
int64 lanelet_id 0
geometry_msgs/Point[2] bound_box


int16 ATTACK_TYPE_NONE=0
int16 ATTACK_TYPE_ANGLE=1
int16 ATTACK_TYPE_DISTANCE=2
int16 ATTACK_TYPE_Z=4
int16 ATTACK_TYPE_RAND_DROP=8
int16 ATTACK_TYPE_LANELET=16
int16 ATTACK_TYPE_BOUND_BOX=32


# ATTACK_TYPE_REQUIRE_LANELET is the addition of all the ATACK_TYPES that require TF
int16 ATTACK_TYPE_REQUIRE_TF=52