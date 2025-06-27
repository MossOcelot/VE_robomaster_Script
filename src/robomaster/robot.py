import threading

from src.robomaster import action
from src.robomaster import logger
from src.robomaster import protocol
from src.robomaster import config
from src.robomaster import conn
from src.robomaster import client

from src.robomaster import chassis

FREE = "free"
GIMBAL_LEAD = "gimbal_lead"
CHASSIS_LEAD = "chassis_lead"
ROBOT_DEFAULT_HOST = protocol.host2byte(9, 6)

class RobotBase:
    def __init__(self, cli=None):
        self._client = cli
        self._modules = {}

    @property
    def client(self):
        return self._client

class Robot(RobotBase):
    _sdk_host = ROBOT_DEFAULT_HOST

    def __init__(self, cli=None):
        super().__init__(cli)
        self._sdk_conn = conn.SdkConnection()
        self._send_heart_beat_timer = None
        self._running = False
        self._initialized = False
        self._conn_type = config.DEFAULT_CONN_TYPE
        self._proto_type = config.DEFAULT_PROTO_TYPE
        # self._ftp = conn.FtpConnection()
        self._modules = {}
        # self._audio_id = 0

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
        return self.get_module("Chassis")
    
    def _scan_modules(self):
        _chassis = chassis.Chassis(self)

        self._modules[_chassis.__class__.__name__] = _chassis

    def get_module(self, name):
        return self._modules[name]
    
    def initialize(self, conn_type=config.DEFAULT_CONN_TYPE, proto_type=config.DEFAULT_PROTO_TYPE, sn=None):
        self._proto_type = proto_type
        self._conn_type = conn_type
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

        # self._ftp.connect(self.ip)

        # start heart beat timer
        self._running = True
        self._start_heart_beat_timer()
        self._initialized = True
        return True
    
    def close(self):
        # self._ftp.stop()
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
    
    def reset(self):
        self._sub_node_reset()
        self._sub_add_node()
        self.set_robot_mode(mode=FREE)
        # self.vision.reset()

    def reset_robot_mode(self):
        proto = protocol.ProtoSetRobotMode()
        proto._mode = 0
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)

        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                return True
            return False
        except Exception as e:
            logger.warning("Robot: set_robot_mode, send_sync_msg exception {0}".format(str(e)))
            return False
        
    def set_robot_mode(self, mode=GIMBAL_LEAD):
        proto = protocol.ProtoSetRobotMode()
        if mode == FREE:
            proto._mode = 0
        elif mode == GIMBAL_LEAD:
            proto._mode = 1
            self.reset_robot_mode()
        elif mode == CHASSIS_LEAD:
            proto._mode = 2
            self.reset_robot_mode()
        else:
            logger.warning("Robot: set_robot_mode, unsupported mode = {0}".format(mode))
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)

        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                return True
            return False
        except Exception as e:
            logger.warning("Robot: set_robot_mode, send_sync_msg exception {0}".format(str(e)))
            return False

    def get_robot_mode(self):
        mode = None
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), protocol.ProtoGetRobotMode())
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto._retcode != 0:
                    raise Exception("get robot mode error.")
                    return None
                if proto._mode == 0:
                    mode = FREE
                elif proto._mode == 1:
                    mode = GIMBAL_LEAD
                elif proto._mode == 2:
                    mode = CHASSIS_LEAD
                else:
                    logger.info("Robot: get_robot_mode, unsupported mode:{0}".format(proto._mode))
                return mode
            else:
                raise Exception('get_robot_mode failed, resp is None.')
        except Exception as e:
            logger.warning("Robot: get_robot_mode, send_sync_msg e {0}".format(e))
            return None
        
    def _enable_sdk(self, enable=1):
        if not self.client:
            return

        proto = protocol.ProtoSetSdkMode()
        proto._enable = enable
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                return True
            else:
                logger.warning("Robot: enable_sdk error.")
                return False
        except Exception as e:
            logger.warning("Robot: enable_sdk, send_sync_msg exception {0}".format(str(e)))
            return False
        
    def get_version(self):
        proto = protocol.ProtoGetProductVersion()
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(8, 1), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                return proto._version
            else:
                logger.warning("Robot: get_version failed.")
                return None
        except Exception as e:
            logger.warning("Robot: get_version, send_sync_msg exception {0}".format(str(e)))
            return None
        
    def get_sn(self):
        proto = protocol.ProtoGetSn()
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(8, 1), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto:
                    return proto._sn
                else:
                    return None
            else:
                logger.warning("Robot: get_sn failed.")
                return None
        except Exception as e:
            logger.warning("Robot: get_sn, send_sync_msg exception {0}".format(str(e)))
            return None
        
    def _sub_add_node(self):
        proto = protocol.ProtoSubscribeAddNode()
        proto._node_id = self._client.hostbyte
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg is not None:
                return True
            else:
                logger.warning("Robot: enable_dds err.")
        except Exception as e:
            logger.warning("Robot: enable_dds, send_sync_msg exception {0}".format(str(e)))
            return False
        
    def _sub_node_reset(self):
        proto = protocol.ProtoSubNodeReset()
        proto._node_id = self._client.hostbyte
        msg = protocol.Msg(self._client.hostbyte, protocol.host2byte(9, 0), proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                return True
            else:
                logger.warning("Robot: reset dds node fail!")
                return False
        except Exception as e:
            logger.warning("Robot: reset_dds, send_sync_msg exception {0}".format(str(e)))
            return False
        
    # def play_audio(self, filename):

    # def play_sound(self, sound_id, times=1)