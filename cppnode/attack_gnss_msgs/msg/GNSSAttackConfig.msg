int16 attack_type 0
float32 accuracy_lat 0
float32 accuracy_lon 0
float32 accuracy_alt 0

float32 drop_chance 0

float32 offset_lat 0
float32 offset_lon 0
float32 offset_alt 0

float32 drift_lat 0
float32 drift_lon 0
float32 drift_alt 0

float32 drift_stop_meter 0
float32 drift_stop_second 0

float32 vel_scale_x 1.0
float32 vel_scale_y 1.0
float32 vel_scale_z 1.0

bool reset_drift true

int16 ATTACK_TYPE_NONE=0
int16 ATTACK_TYPE_ACCURACY=1
int16 ATTACK_TYPE_DROP=2
int16 ATTACK_TYPE_OFFSET=4
int16 ATTACK_TYPE_DRIFT=8
int16 ATTACK_TYPE_DRIFT_STOP_DIST=16
int16 ATTACK_TYPE_DRIFT_STOP_TIME=32
int16 ATTACK_TYPE_VEL_SCALE=64
