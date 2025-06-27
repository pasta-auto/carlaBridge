#!/usr/bin/env python3

import json
import subprocess
import time


TEST_CASES = [
    "--replaypercent 1.0 --replaycount 1 --replaydelay 1000",
    "--replaypercent 0.5 --replaycount 1 --replaydelay 1000",
    "--replaypercent 0.5 --replaycount 2 --replaydelay 1000",
    "--replaypercent 1.0 --replaycount 2 --replaydelay 500",
    "--dospercent 0.5",
    "--doscount 2",
]
ATTACK_TARGETS = {
    "camera": "/sensing/camera/camera0/image_rect_color",
    "camera_info": "/sensing/camera/camera0/camera_info",
    "lidar": "/sensing/lidar/top/outlier_filtered/pointcloud",
    "nav_sat": "/sensing/gnss/ublox/nav_sat_fix",
    "ecef_vel": "/sensing/gnss/ecef_twist_with_covariance",
    "imu": "/sensing/imu/tamagawa/imu_raw",
    "gnss_ins": "/autoware_orientation",
    "steering_status": "/vehicle/status/steering_status",
    "hazard_status": "/vehicle/status/hazard_lights_status",
    "turning_status": "/vehicle/status/turn_indicators_status",
    "gear_report": "/vehicle/status/gear_status",
    "velocity_report": "/vehicle/status/velocity_status",
    "actuation_status": "/vehicle/status/actuation_status",
    "handbrake_status": "/vehicle/status/handbrake_status",
    "twist": "/sensing/vehicle_velocity_converter/twist_with_covariance",
}
SCRIPT_PATH = "./scripts/ethernet_attack_control.sh"


def run_attack_cmd(cmd):
    subprocess.run(cmd.split(), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.run(cmd.split(), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)


def test_attack(args, sample_time=10):
    result = {}
    for name, path in ATTACK_TARGETS.items():
        attack_command = f"{SCRIPT_PATH} --target {name} {args}"
        reset_cmd = f"{SCRIPT_PATH} --reset"
        echo_command = f"ros2 topic echo {path}"
        print(f"Test command: {attack_command}")

        count = {}
        msg_counts = []

        def capture_echo_output():
            start_time = int(time.time())
            with subprocess.Popen(echo_command.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT) as process:
                for line in process.stdout:
                    if int(time.time()) - start_time >= sample_time:
                        process.terminate()
                    content = line.decode("utf8").strip().split()
                    if content[0] == "sec:":
                        if content[1] not in count:
                            count[content[1]] = 0
                        count[content[1]] += 1

                msg_counts.append([c for _, c in count.items()])
                count.clear()

        run_attack_cmd(reset_cmd)
        capture_echo_output()
        run_attack_cmd(attack_command)
        capture_echo_output()
        run_attack_cmd(reset_cmd)

        result[name] = msg_counts
    return result


if __name__ == "__main__":
    result = {}
    try:
        # Test involves a lot of waiting so provide an estimated completion time
        # Estimated time assumes each test case/target combo takes 30 seconds
        end_time = time.time() + (len(TEST_CASES) * len(ATTACK_TARGETS) * 30)
        end_time_fmt = time.strftime("%H:%M", time.localtime(end_time))
        print("===============================")
        print(f"Test will finish around: {end_time_fmt}")
        print("===============================")

        for case in TEST_CASES:
            result[case] = test_attack(case)
    except KeyboardInterrupt:
        print("Testing interrupted. Outputting current test results")
    finally:
        result_json_str = json.dumps(result, indent=2)
        with open("test_result.json", "w") as f:
            f.write(result_json_str)
        print(result_json_str)
