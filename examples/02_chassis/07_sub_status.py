import time
import robomaster
from robomaster import robot


def sub_status_info_handler(status_info):
    static_flag, up_hill, down_hill, on_slope, pick_up, slip_flag, impact_x, impact_y, impact_z, \
    roll_over, hill_static = status_info
    print("chassis status: static_flag:{0}, up_hill:{1}, down_hill:{2}, on_slope:{3}, "
          "pick_up:{4}, impact_x:{5}, impact_y:{6}, impact_z:{7}, roll_over:{8}, "
          "hill_static:{9}".format(static_flag, up_hill, down_hill, on_slope, pick_up,
                                   slip_flag, impact_x, impact_y, impact_z, roll_over, hill_static))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    ep_chassis.sub_status(freq=5, callback=sub_status_info_handler)
    time.sleep(3)
    ep_chassis.unsub_status()

    ep_robot.close()