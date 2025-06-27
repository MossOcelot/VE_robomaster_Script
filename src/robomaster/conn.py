import binascii
import random
import socket
import traceback

from src.robomaster import config
from src.robomaster import protocol
from src.robomaster import logger

CONNECTION_LOCAL_ENV = 'local_env'
CONNECTION_PROTO_TCP = 'tcp'
CONNECTION_PROTO_UDP = 'udp'

class BaseConnection:
    def __init__(self):
        self._sock = None
        self._buf = bytearray()
        self._host_addr = None
        self._target_addr = None
        self._proto_type = None
        self._proto = None

    def create(self):
        """Create a socket connection."""
        try:
            if self._proto_type == CONNECTION_PROTO_UDP:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

                # Bind socket to local IP address and port
                # This allows the socket to receive data sent to this address
                # and sets the source address when sending data
                self._sock.bind(self._host_addr) # define the local address to bind to
                logger.info("UdpConnection, bind {0}".format(self._host_addr))
            else:
                logger.error("Connection: {0} unexpected connection param set".format(self._proto_type))

        except Exception as e:
            logger.warning("udpConnection: create, host_addr:{0}, exception:{1}".format(self._host_addr, e))
            raise

    def close(self):
        if self._sock:
            self._sock.close()

    def recv(self):
        try:
            if self._sock:
                data, host = self._sock.recvfrom(2048)
        except Exception as e:
            logger.warning("Connection: recv, exception:{0}".format(e))
            raise

        if data is None:
            logger.warning("Connection: recv buff None.")
            return None
        
        self._buf.extend(data)
        if len(self._buf) == 0:
            logger.warning("Connection: recv buff None.")
            return None
        
        msg, self._buf = protocol.decode_msg(self._buf, self._proto)
        if not msg:
            logger.warning("Connection: protocol.decode_msg is None.")
            return None
        else:
            if isinstance(msg, protocol.MsgBase):
                if not msg.unpack_protocol():
                    logger.warning("Connection: recv, msg.unpack_protocol failed, msg:{0}".format(msg))
            return msg
        
    def send(self, buf):
        try:
            if self._sock:
                self._sock.sendto(buf, self._target_addr)
        except Exception as e:
            logger.warning("Connection: send, exception:{0}".format(e))
            raise

    def send_self(self, buf):
        try:
            if self._sock:
                self._sock.sendto(buf, self._host_addr)
        except Exception as e:
            logger.warning("Connection: send, exception:{0}".format(e))
            raise

class Connection(BaseConnection):
    def __init__(self, host_addr, target_addr, proto="v1", protocol=CONNECTION_PROTO_UDP):
        self._host_addr = host_addr
        self._target_addr = target_addr
        self._proto = proto
        self._proto_type = protocol

        self._sock = None
        self._buf = bytearray()

    def __repr__(self):
        return "Connection, host:{0}, target:{1}".format(self._host_addr, self._target_addr)

    @property
    def target_addr(self):
        return self._target_addr

    @property
    def protocol(self):
        return self._proto

class SdkConnection(BaseConnection):
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def __del__(self):
        self.close()

    def switch_remote_route(self, msg, remote_addr, timeout=5):
        if not self._sock:
            return False, None
        
        buf = msg.pack()
        logger.debug("SdkConnection, switch_remote_route, bug:{0}, remote_addr:{1}.".format(buf, remote_addr))
        try:
            self._sock.settimeout(timeout)
            self._sock.sendto(buf, remote_addr)
            data, address = self._sock.recvfrom(1024)
            self._sock.settimeout(timeout)
            logger.debug("SdkConnection, data:{0}.".format(binascii.hexlify(data)))
            resp_msg, data = protocol.decode_msg(data)
            resp_msg.unpack_protocol()
            if resp_msg:
                prot = resp_msg.get_proto()
                if prot._retcode == 0:
                    if prot._state == 0:
                        logger.info("SdkConnection: accept connection.")
                    if prot._state == 1:
                        logger.error("SdkConnection: reject connection, service is busy!")
                        return False, None
                    if prot._state == 2:
                        logger.info("SdkConnection: got config ip:{0}".format(prot._config_ip))
                        return True, prot._config_ip
        except socket.timeout:
            logger.error("SdkConnection: RECV TimeOut!")
            raise
        except Exception as e:
            logger.warning("SdkConnection: switch_remote_route, exception:{0}, Please Check Connections.".format(e))
            logger.warning("SdkConnection:{0}".format(traceback.format_exc()))
            return False, None
        
    def request_connection(self, sdk_host, conn_type=None, proto_type=None, sn=None):
        if conn_type is None:
            logger.error("Not Specific conn_type!")
        logger.info("CONN TYPE is {0}".format(conn_type))
        local_addr = None
        remote_addr = None
        proto = protocol.ProtoSetSdkConnection()
        if conn_type == CONNECTION_LOCAL_ENV:
            proto._connection = 0
            
            # localhost 
            proto._ip = '0.0.0.0' 
            logger.info("Robot: request_connection, ap get local ip:{0}".format(proto._ip))
            proto._port = random.randint(config.ROBOT_SDK_PORT_MIN, config.ROBOT_SDK_PORT_MAX)

            remote_addr = config.ENV_ROBOT_DEFAULT_ADDR
            local_addr = (proto._ip, proto._port)

        logger.info("SdkConnection: request_connection, local addr {0}, remote_addr {1}, ".format(local_addr, remote_addr))

        proto._host = sdk_host
        if proto_type == CONNECTION_PROTO_TCP:
            proto._protocol = 1
        else:
            proto._protocol = 0

        msg = protocol.Msg(sdk_host, protocol.host2byte(9, 0), proto)
        try:
            result, local_ip = self.switch_remote_route(msg, remote_addr)
            if result:
                local_addr = (local_ip, proto._port)
            else:
                return False, local_addr, remote_addr
            return result, local_addr, remote_addr
        
        except socket.timeout:
            logger.warning("SdkConnection: Connection Failed, please check hareware connections!!!")
            return False, local_addr, remote_addr
        except Exception as e:
            logger.warning("SdkConnection: request_connection, switch_remote_route exception {0}".format(str(e)))
            return False, local_addr, remote_addr