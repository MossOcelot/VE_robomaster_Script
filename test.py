from src.robot import VirtualRobot

if __name__ == '__main__':
    # Create an instance of the VirtualRobot class
    robot = VirtualRobot()
    robot.set_simulation_config(host='localhost', port=5000)
    robot.initialize(conn_type='sim')