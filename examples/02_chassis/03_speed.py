import time
from src.robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="local_env")

    ep_chassis = ep_robot.chassis

    x_val = 1
    y_val = 0.3
    z_val = 30

    # Move forward for 3 seconds
    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # Move backward for 3 seconds
    ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
    time.sleep(3)

    # Move left for 3 seconds
    ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5) # left
    time.sleep(3)

    # Move right for 3 seconds
    ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5) # right
    time.sleep(3)

    # Turn left for 3 seconds
    ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5) # left rotate
    time.sleep(3)

    # Turn right for 3 seconds
    ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5) # right rotate
    time.sleep(3)

    # Stop mecanum wheel movement
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop

    ep_robot.close()
