# Ethernet Attack

Usage: `./ethernet_attack_control.sh --target <targets> <options>`

## Supported targets
- all
- camera
- camera_info
- lidar
- nav_sat
- ecef_vel
- imu
- gnss_ins
- steering_status
- hazard_status
- turning_status
- gear_report
- velocity_report
- actuation_status
- handbrake_status
- headlights_status
- twist

## Options
### Simulating a Replay Attack
- `--replaypercent`
    - The percentage of messages to capture and replay for the given target.
- `--replaydelay`
    - The amount of time (in milliseconds) to wait before replaying the message.
- `--replaycount`
    - The number of times to replay each captured message.
    - If replaycount > 1, the message will be played once every `replaydelay` milliseconds

### Simulating a DoS Attack on the Network Switch
- `--dosdelay`
    - The amount of time (in milliseconds) to delay each incoming message
- `--dospercent`
    - The percentage of messages to drop completely

### Simulating a DoS Attack on Individual Hosts
- `--doscount`
    - The number of times to duplicate each incoming messages
    - Duplicated messages will be sent out immediately
    - Note: Due to limitations with ROS, the actual number of duplicated messages will sometimes be less than expected as each publisher requires some amount of time to finish sending a message before another one can be successfully published. This disprecancy will be particularly noticable for larger message types like the LiDAR pointcloud or for very high values of `doscount`.


## Example Usage
- `./ethernet_attack_control.sh --target all --dosdelay 500`
- `./ethernet_attack_control.sh --target lidar,nav_sat,twist --replaypercent 0.05 --replaycount 1 --replaydelay 1000`
