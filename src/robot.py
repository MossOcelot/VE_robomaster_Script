import threading
from robomaster import robot
from robomaster.robot import protocol
from robomaster.robot import action
from robomaster.robot import logger
from robomaster.robot import client
from robomaster.robot import conn
from robomaster.robot import gimbal
from robomaster.robot import chassis
from robomaster.robot import camera
from robomaster.robot import blaster
from robomaster.robot import vision
from robomaster.robot import servo
from robomaster.robot import config
from robomaster.robot import dds
from robomaster.robot import led
from robomaster.robot import battery
from robomaster.robot import robotic_arm
from robomaster.robot import sensor
from robomaster.robot import gripper
from robomaster.robot import armor
from robomaster.robot import flight
from robomaster.robot import uart
from robomaster.robot import ai_module

from src import new_conn

ROBOT_DEFAULT_HOST = protocol.host2byte(9, 6)

class SimulationConfig:
    def __init__(self, host=ROBOT_DEFAULT_HOST, port=5000):
        self.host = host
        self.port = port

    def __str__(self):
        return "SimulationConfig(host={0}, port={1})".format(self.host, self.port)

class VirtualRobot(robot.RobotBase):
    _product = "EP"
    _sdk_host = ROBOT_DEFAULT_HOST
    
    def __init__(self, cli=None):
        self._config = config.ep_conf
        super().__init__(cli)
        self._sdk_conn = conn.SdkConnection()
        self._sim_conn = new_conn.SimConnection()
        self._send_heart_beat_timer = None
        self._running = False
        self._initialized = False
        self._conn_type = config.DEFAULT_CONN_TYPE
        self._proto_type = config.DEFAULT_PROTO_TYPE
        self._ftp = conn.FtpConnection()
        self._modules = {}
        self._audio_id = 0
        self._simulation_config = SimulationConfig("192.168.1.1", 5000)

    def __del__(self):
        self.close()

        if self is None:
            return

        for name in list(self._modules.keys()):
            if self._modules[name]:
                del self._modules[name]

    def _start_heart_beat_timer(self):
        if self._running:
            self._send_heart_beat_msg()

    def _stop_heart_beat_timer(self):
        if self._send_heart_beat_timer:
            self._send_heart_beat_timer.cancel()
            self._send_heart_beat_timer = None

    def _send_heart_beat_msg(self):
        proto = protocol.ProtoSdkHeartBeat()
        msg = protocol.Msg(self.client.hostbyte, protocol.host2byte(9, 0), proto)
        try:
            self.client.send_msg(msg)
        except Exception as e:
            logger.warning("Robot: send heart beat msg failed, exception {0}".format(e))
        if self._running:
            self._send_heart_beat_timer = threading.Timer(1, self._send_heart_beat_msg)
            self._send_heart_beat_timer.start()
    
    def set_simulation_config(self, host=None, port=None):
        if host:
            self._simulation_config.host = host
        if port:
            self._simulation_config.port = port
        return self._simulation_config

    @property
    def conf(self):
        return self._config

    @property
    def action_dispatcher(self):
        return self._action_dispatcher

    @property
    def ip(self):
        return self.client.remote_addr[0]

    @property
    def conn_type(self):
        return self._conn_type

    @property
    def proto_type(self):
        return self._proto_type

    @property
    def chassis(self):
        """ 获取底盘模块对象 """
        return self.get_module("Chassis")

    @property
    def gimbal(self):
        """ 获取云台模块对象 """
        return self.get_module("Gimbal")

    @property
    def blaster(self):
        """ 获取水弹枪模块对象 """
        return self.get_module("Blaster")

    @property
    def led(self):
        """ 获取灯效控制模块对象 """
        return self.get_module("Led")

    @property
    def vision(self):
        """ 获取智能识别模块对象 """
        return self.get_module("Vision")

    @property
    def battery(self):
        """ 获取电池模块对象 """
        return self.get_module("Battery")

    @property
    def camera(self):
        """ 获取相机模块对象 """
        return self.get_module("EPCamera")

    @property
    def robotic_arm(self):
        """ 获取机械臂模块对象 """
        return self.get_module("RoboticArm")

    @property
    def dds(self):
        return self.get_module("Subscriber")

    @property
    def servo(self):
        return self.get_module("Servo")

    @property
    def sensor(self):
        return self.get_module("DistanceSensor")

    @property
    def sensor_adaptor(self):
        return self.get_module("SensorAdaptor")

    @property
    def gripper(self):
        return self.get_module("Gripper")

    @property
    def armor(self):
        return self.get_module("Armor")

    @property
    def uart(self):
        return self.get_module("Uart")

    @property
    def ai_module(self):
        return self.get_module("AiModule")

    @property
    def is_initialized(self):
        return self._initialized

    def _scan_modules(self):
        _gimbal = gimbal.Gimbal(self)
        _chassis = chassis.Chassis(self)
        _camera = camera.EPCamera(self)
        _blaster = blaster.Blaster(self)
        _vision = vision.Vision(self)
        _dds = dds.Subscriber(self)
        _dds.start()
        _led = led.Led(self)
        _battery = battery.Battery(self)
        _servo = servo.Servo(self)
        _dis_sensor = sensor.DistanceSensor(self)
        _sensor_adaptor = sensor.SensorAdaptor(self)
        _robotic_arm = robotic_arm.RoboticArm(self)
        _gripper = gripper.Gripper(self)
        _armor = armor.Armor(self)
        _uart = uart.Uart(self)
        _uart.start()
        _ai_module = ai_module.AiModule(self)

        self._modules[_gimbal.__class__.__name__] = _gimbal
        self._modules[_chassis.__class__.__name__] = _chassis
        self._modules[_camera.__class__.__name__] = _camera
        self._modules[_blaster.__class__.__name__] = _blaster
        self._modules[_vision.__class__.__name__] = _vision
        self._modules[_dds.__class__.__name__] = _dds
        self._modules[_led.__class__.__name__] = _led
        self._modules[_battery.__class__.__name__] = _battery
        self._modules[_servo.__class__.__name__] = _servo
        self._modules[_robotic_arm.__class__.__name__] = _robotic_arm
        self._modules[_dis_sensor.__class__.__name__] = _dis_sensor
        self._modules[_sensor_adaptor.__class__.__name__] = _sensor_adaptor
        self._modules[_gripper.__class__.__name__] = _gripper
        self._modules[_armor.__class__.__name__] = _armor
        self._modules[_uart.__class__.__name__] = _uart
        self._modules[_ai_module.__class__.__name__] = _ai_module

    def get_module(self, name):
        """ 获取模块对象

        :param name: 模块名称，字符串，如：chassis, gimbal, led, blaster, camera, battery, vision, etc.
        :return: 模块对象
        """
        return self._modules[name]
    
    def initialize(self, conn_type=config.DEFAULT_CONN_TYPE, proto_type=config.DEFAULT_PROTO_TYPE, sn=None):
        """ Initialize the robot

        :param conn_type: Connection type: 'ap' means direct hotspot connection; 'sta' means networked connection; 'rndis' means USB connection, 'sim' means simulation mode.
        :param proto_type: Communication protocol: 'tcp', 'udp'

        Note: To modify the default connection type, you can specify DEFAULT_CONN_TYPE in conf.py
        """

        self._proto_type = proto_type
        self._conn_type = conn_type

        if conn_type == "sim":
            logger.info("Robot: Simulation mode, using SimulationConfig {0}".format(self._simulation_config))
            sim_conn1 = self._wait_for_simulation_connection(self._simulation_config.host, self._simulation_config.port, proto_type)

            return True
        if not self._client:
            logger.info("Robot: try to connection robot.")
            conn1 = self._wait_for_connection(conn_type, proto_type, sn)
            if conn1:
                logger.info("Robot: initialized with {0}".format(conn1))
                self._client = client.Client(9, 6, conn1)
            else:
                logger.info("Robot: initialized, try to use default Client.")
                try:
                    self._client = client.Client(9, 6)
                except Exception as e:
                    logger.error("Robot: initialized, can not create client, return, exception {0}".format(e))
                    return False

        try:
            self._client.start()
        except Exception as e:
            logger.error("Robot: Connection Create Failed.")
            raise e

        self._action_dispatcher = action.ActionDispatcher(self.client)
        self._action_dispatcher.initialize()
        # Reset Robot, Init Robot Mode.
        self._scan_modules()

        # set sdk mode and reset
        self._enable_sdk(1)
        self.reset()

        self._ftp.connect(self.ip)

        # start heart beat timer
        self._running = True
        self._start_heart_beat_timer()
        self._initialized = True
        return True
    
    def close(self):
        self._ftp.stop()
        if self._initialized:
            self._enable_sdk(0)
            self._stop_heart_beat_timer()
        for name in list(self._modules.keys()):
            if self._modules[name]:
                self._modules[name].stop()
        if self.client:
            self._client.stop()
        if self._sdk_conn:
            self._sdk_conn.close()
        self._initialized = False
        logger.info("Robot close")

    def _wait_for_connection(self, conn_type, proto_type, sn=None):
        result, local_addr, remote_addr = self._sdk_conn.request_connection(self._sdk_host, conn_type, proto_type, sn)
        if not result:
            logger.error("Robot: Connection Failed, Please Check Hareware Connections!!! "
                         "conn_type {0}, host {1}, target {2}.".format(conn_type, local_addr, remote_addr))
            return None
        return conn.Connection(local_addr, remote_addr, protocol=proto_type)
    
    def _wait_for_simulation_connection(self, host, port, proto_type):
        """ Wait for a simulation connection """
        self._sim_conn.request_connection(host, port, proto_type)


    