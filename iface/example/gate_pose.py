################################################################################
# Gate task (pose feedback).
# Uses SASSIST2 mode to descend to 1 meter depth, hold that depth for 1
# second, then travel accurate X/Y distances using the ZED camera pose
# published on the ROS2 topic /zed/zed_node/pose (geometry_msgs/PoseStamped).
#
# Pose frame (as published): +X right, +Y forward, +Z up. This maps directly
# to SASSIST2 x (strafe) / y (forward) as long as the initial heading is held,
# which this script does.
#
# Requires ROS2 (rclpy). ROS is not installed on the host, so run this via
# the docker wrapper: example/gate_pose_docker.sh
################################################################################

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import threading
import time


DEPTH_TARGET = -1.0     # meters (negative = below surface)
DEPTH_TOL = 0.1         # meters
DEPTH_TIMEOUT = 30.0    # seconds to reach target depth before aborting
DEPTH_HOLD_TIME = 1.0   # seconds
PERIOD = 0.05           # control loop period (speed sets feed motor watchdog)

TARGET_X = 0.0          # meters to travel right (negative = left)
TARGET_Y = 4.0          # meters to travel forward (negative = backward)
POS_TOL = 0.1           # meters
KP = 0.5                # proportional gain (speed per meter of error)
MAX_SPEED = 0.5         # translation speed limit
POSE_WAIT = 5.0         # seconds to wait for first pose message
POSE_STALE = 1.0        # abort if no pose received for this long mid-move

POSE_TOPIC = "/zed/zed_node/pose"


# Latest pose shared between rclpy spin thread and control loop
_pose_lock = threading.Lock()
_pose = None            # (x, y, z)
_pose_time = 0.0


def on_pose(msg):
    global _pose, _pose_time
    pos = msg.pose.position
    with _pose_lock:
        _pose = (pos.x, pos.y, pos.z)
        _pose_time = time.time()


def get_pose():
    with _pose_lock:
        return _pose, _pose_time


def clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def stop(cb: ControlBoard):
    cb.set_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def goto_relative(cb: ControlBoard, dx: float, dy: float, yaw: float) -> bool:
    start_pose, _ = get_pose()
    goal_x = start_pose[0] + dx
    goal_y = start_pose[1] + dy
    print("Moving dx={:.2f}m dy={:.2f}m...".format(dx, dy), end="", flush=True)
    while True:
        pose, pose_time = get_pose()
        if time.time() - pose_time > POSE_STALE:
            print("Fail. Pose data stale.")
            stop(cb)
            return False
        err_x = goal_x - pose[0]
        err_y = goal_y - pose[1]
        if abs(err_x) < POS_TOL and abs(err_y) < POS_TOL:
            print("Done.")
            return True
        cb.set_sassist2(clamp(KP * err_x, MAX_SPEED), clamp(KP * err_y, MAX_SPEED),
                        0.0, 0.0, yaw, DEPTH_TARGET)
        time.sleep(PERIOD)


def run(cb: ControlBoard, s: Simulator) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped

    print("Query sensor status...", end="")
    res, bno055, ms5837 = cb.get_sensor_status()
    if res != ControlBoard.AckError.NONE or not bno055 or not ms5837:
        print("Fail. BNO055 ready: {}, MS5837 ready: {}".format(bno055, ms5837))
        return 1
    print("Done.")

    print("Enable periodic sensor data...", end="")
    if cb.read_bno055_periodic(True) != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    if cb.read_ms5837_periodic(True) != ControlBoard.AckError.NONE:
        print("Fail.")
        return 1
    print("Done.")
    time.sleep(0.5)  # Wait for data to actually be sent

    print("Subscribing to {}...".format(POSE_TOPIC), end="", flush=True)
    rclpy.init()
    node = Node("gate_pose")
    # Best-effort subscriber is QoS-compatible with both reliable and
    # best-effort publishers, so this works regardless of the ZED node's QoS
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        start = time.time()
        while get_pose()[0] is None:
            if time.time() - start > POSE_WAIT:
                print("Fail. No pose data (is the ZED node running?).")
                return 1
            time.sleep(0.1)
        print("Done.")

        # Hold whatever heading the vehicle starts with
        initial_yaw = cb.get_bno055_data().yaw
        print("Holding yaw: {:.1f}".format(initial_yaw))

        # Descend to target depth
        print("Descending to {:.2f}m...".format(DEPTH_TARGET), end="", flush=True)
        start = time.time()
        while abs(cb.get_ms5837_data().depth - DEPTH_TARGET) > DEPTH_TOL:
            if time.time() - start > DEPTH_TIMEOUT:
                print("Fail. Timed out at depth {:.2f}m.".format(
                    cb.get_ms5837_data().depth))
                stop(cb)
                return 1
            cb.set_sassist2(0.0, 0.0, 0.0, 0.0, initial_yaw, DEPTH_TARGET)
            time.sleep(PERIOD)
        print("Done.")

        # Hold depth
        print("Holding depth for {:.1f}s...".format(DEPTH_HOLD_TIME), end="", flush=True)
        start = time.time()
        while time.time() - start < DEPTH_HOLD_TIME:
            cb.set_sassist2(0.0, 0.0, 0.0, 0.0, initial_yaw, DEPTH_TARGET)
            time.sleep(PERIOD)
        print("Done.")

        # Travel to target using pose feedback
        if not goto_relative(cb, TARGET_X, TARGET_Y, initial_yaw):
            return 1

        print("Stopping...", end="")
        stop(cb)
        print("Done.")
        return 0
    except (KeyboardInterrupt, Exception) as e:
        stop(cb)
        if not isinstance(e, KeyboardInterrupt):
            raise e
        return 1
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
