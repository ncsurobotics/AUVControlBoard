if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import time


DEPTH_TARGET = -1.0     # meters (negative = below surface)
DEPTH_TOL = 0.1         # meters
DEPTH_TIMEOUT = 30.0    # seconds to reach target depth before aborting
DEPTH_HOLD_TIME = 1.0   # seconds
FORWARD_SPEED = 0.5     # -1.0 to 1.0
FORWARD_TIME = 4.0      # seconds
PERIOD = 0.05           # control loop period (speed sets feed motor watchdog)


def stop(cb: ControlBoard):
    cb.set_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def run(cb: ControlBoard, s: Simulator) -> int:
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

    # Hold whatever heading the vehicle starts with
    initial_yaw = cb.get_bno055_data().yaw
    print("Holding yaw: {:.1f}".format(initial_yaw))

    try:
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

        # Drive forward
        print("Forward for {:.1f}s...".format(FORWARD_TIME), end="", flush=True)
        start = time.time()
        while time.time() - start < FORWARD_TIME:
            cb.set_sassist2(0.0, FORWARD_SPEED, 0.0, 0.0, initial_yaw, DEPTH_TARGET)
            time.sleep(PERIOD)
        print("Done.")

        print("Stopping...", end="")
        stop(cb)
        print("Done.")
        return 0
    except (KeyboardInterrupt, Exception) as e:
        stop(cb)
        if not isinstance(e, KeyboardInterrupt):
            raise e
        return 1
