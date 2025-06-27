
import time
from src.robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="local_env")

    ep_chassis = ep_robot.chassis

    speed = 50
    slp = 1

    # forward (front_right, front_left, rear_left, rear_right)
    ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
    time.sleep(slp)
    # backward (front_right, front_left, rear_left, rear_right)
    ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
    time.sleep(slp)
    # left (front_right, front_left, rear_left, rear_right) 
    ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=speed, w4=-speed)
    time.sleep(slp)
    # right (front_right, front_left, rear_left, rear_right)
    ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=-speed, w4=speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=speed, w2=0, w3=speed, w4=0) 
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=0, w2=speed, w3=0, w4=speed)
    time.sleep(slp)
    
    ep_chassis.drive_wheels(w1=0, w2=-speed, w3=0, w4=-speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=-speed, w2=0, w3=-speed, w4=0)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
    time.sleep(slp)
    ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
    time.sleep(slp)
    
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    ep_robot.close()