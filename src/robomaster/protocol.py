from abc import abstractmethod
import binascii
import struct
from src.robomaster import logger
from src.robomaster import algo

# 默认的 ID 取值范围
RM_SDK_FIRST_SEQ_ID = 10000
RM_SDK_LAST_SEQ_ID = 20000

# 协议 ACK 类型
DUSS_MB_ACK_NO = 0
DUSS_MB_ACK_NOW = 1
DUSS_MB_ACK_FINISH = 2

# 协议加密类型
DUSS_MB_ENC_NO = 0
DUSS_MB_ENC_AES128 = 1
DUSS_MB_ENC_CUSTOM = 2

# 协议类型
DUSS_MB_TYPE_REQ = 0
DUSS_MB_TYPE_PUSH = 1

def host2byte(host, index):
    return index * 32 + host


def byte2host(b):
    return (b & 0x1f), (b >> 5)


def make_proto_cls_key(cmdset, cmdid):
    return cmdset * 256 + cmdid


# registered protocol dict.
registered_protos = {}

class _AutoRegisterProto(type):
    """ help to automatically register Proto Class where ever they're defined """

    def __new__(mcs, name, bases, attrs, **kw):
        return super().__new__(mcs, name, bases, attrs, **kw)

    def __init__(cls, name, bases, attrs, **kw):
        super().__init__(name, bases, attrs, **kw)
        if name == 'ProtoData':
            return
        key = make_proto_cls_key(attrs['_cmdset'], attrs['_cmdid'])
        if key in registered_protos.keys():
            raise ValueError("Duplicate proto class %s" % (name))
        registered_protos[key] = cls

class ProtoData(metaclass=_AutoRegisterProto):
    _cmdset = None
    _cmdid = None
    _cmdtype = DUSS_MB_TYPE_REQ
    _req_size = 0
    _resp_size = 0

    def __init__(self, **kwargs):
        self._buf = None
        self._len = None
    
    def __repr__(self):
        return "<{0} cmset:0x{1:2x}, cmdid:0x{2:02x}>".format(self.__class__.__name__, self._cmdset, self._cmdid)

    @property
    def cmdset(self):
        return self._cmdset

    @cmdset.setter
    def cmdset(self, value):
        self._cmdset = value

    @property
    def cmdid(self):
        return self._cmdid

    @cmdid.setter
    def cmdid(self, value):
        self._cmdid = value

    @property
    def cmdkey(self):
        if self._cmdset is not None and self._cmdid is not None:
            return self._cmdset * 256 + self._cmdid
        else:
            return None
        
    @abstractmethod
    def pack_req(self):
        return b''
    
    def unpack_req(self, buf, offset=0):
        """ 从字节流解包

        :param buf：字节流数据
        :param offset：字节流数据偏移量
        :return：True 解包成功；False 解包失败
        """
        return True
    
    def pack_resp(self):
        """ 协议对象打包

        :return：字节流数据
        """
        pass

    def unpack_resp(self, buf, offset=0):
        """ 从字节流解包为返回值和相关属性

        :param buf：字节流数据
        :param offset：字节流数据偏移量
        :return: bool: 调用结果
        """
        self._retcode = buf[offset]
        if self._retcode == 0:
            return True
        else:
            return False
    

class MsgBase:
    _next_seq_id = RM_SDK_FIRST_SEQ_ID

    def __init__(self):
        pass

class Msg(MsgBase):
    def __init__(self, sender=0, receiver=0, proto=None):
        self._len = 13 # default length, msg header and crc.
        self._sender = sender
        self._receiver = receiver
        self._attri = 0
        self._cmdset = None
        self._cmdid = None

        self._is_ack = False  # True or False
        self._need_ack = 2  # 0 for no need, 1 for ack now, 2 for need when finish.
        if self.__class__._next_seq_id == RM_SDK_LAST_SEQ_ID:
            self.__class__._next_seq_id = RM_SDK_FIRST_SEQ_ID
        else:
            self.__class__._next_seq_id += 1
        self._seq_id = self._next_seq_id
        self._proto = proto
        if self._proto:
            self._cmdset = self._proto.cmdset
            self._cmdid = self._proto.cmdid
            if self._proto._cmdtype == DUSS_MB_TYPE_PUSH:
                self._need_ack = 0
        self._buf = None

    def __repr__(self):
        return "<Msg sender:0x{0:02x}, receiver:0x{1:02x}, cmdset:0x{2:02x}, cmdid:0x{3:02x}, len:{4:d}, \
seq_id:{5:d}, is_ack:{6:d}, need_ack:{7:d}>".format(self._sender, self._receiver, self._cmdset, self._cmdid,
                                                    self._len, self._seq_id, self._is_ack, self._need_ack)
    
    @property
    def cmdset(self):
        return self._cmdset

    @property
    def cmdid(self):
        return self._cmdid

    @property
    def is_ack(self):
        return self._is_ack

    @property
    def receiver(self):
        host, index = byte2host(self._receiver)
        return "{0:02d}{1:02d}".format(host, index)

    @property
    def sender(self):
        host, index = byte2host(self._sender)
        return "{0:02d}{1:02d}".format(host, index)
    
    def pack(self, is_ack=False):
        self._len = 13
        try:
            if self._proto:
                data_buf = b''
                if is_ack:
                    self._neek_ack = False
                    data_buf = self._proto.pack_resp()
                else:
                    self._neek_ack = (self._proto._cmdtype == DUSS_MB_TYPE_REQ)
                    data_buf = self._proto.pack_req()
                self._len += len(data_buf)
        except Exception as e:
            logger.warning("Msg: pack, cmset:0x{0:02x}, cmdid:0x{1:02x}, proto: {2}, "
                           "exception {3}".format(self.cmdset, self.cmdid, self._proto.__class__.__name__, e))

        self._buf = bytearray(self._len)
        self._buf[0] = 0x55
        self._buf[1] = self._len & 0xff
        self._buf[2] = (self._len >> 8) & 0x3 | 4
        crc_h = algo.crc8_calc(self._buf[0:3])

        # attri = is_ack|need_ack|enc
        self._attri = 1 << 7 if self._is_ack else 0
        self._attri += self._need_ack << 5
        self._buf[3] = crc_h
        self._buf[4] = self._sender
        self._buf[5] = self._receiver
        self._buf[6] = self._seq_id & 0xff
        self._buf[7] = (self._seq_id >> 8) & 0xff
        self._buf[8] = self._attri

        if self._proto:
            self._buf[9] = self._proto.cmdset
            self._buf[10] = self._proto.cmdid
            self._buf[11:11 + len(data_buf)] = data_buf
        else:
            raise Exception("Msg: pack Error.")

        # calc whole msg crc16
        crc_m = algo.crc16_calc(self._buf[0:self._len - 2])
        struct.pack_into('<H', self._buf, self._len - 2, crc_m)

        logger.debug("Msg: pack, len:{0}, seq_id:{1}, buf:{2}".format(
            self._len, self._seq_id, binascii.hexlify(self._buf)))
        return self._buf
    
    def unpack_protocol(self):
        key = make_proto_cls_key(self._cmdset, self._cmdid)
        if key in registered_protos.keys():
            self._proto = registered_protos[key]()
            try:
                if self._is_ack:
                    if not self._proto.unpack_resp(self._buf):
                        logger.warning("Msg: unpack_protocol, msg:{0}".format(self))
                        return False
                else:
                    if not self._proto.unpack_req(self._buf):
                        logger.warning("Msg: unpack_protocol, msg:{0}".format(self))
                        return False
                return True
            except Exception as e:
                logger.warning("Msg: unpack_protocol, {0} failed e {1}".format(self._proto.__class__.__name__, e))
                raise
        else:
            logger.info("Msg: unpack_protocol, cmdset:0x{0:02x}, cmdid:0x{1:02x}, class is not registerin registered_\
protos".format(self._cmdset, self._cmdid))
            pass
        logger.warning("Msg: unpack_protocol, not registered_protocol, cmdset:0x{0:02x}, cmdid:0x{1:02x}".format(
            self._cmdset, self._cmdid))
        return False
    
    def get_proto(self):
        return self._proto

class TextMsg(MsgBase):
    IS_DDS_FLAG = ";mpry:"

    def __init__(self, proto=None):
        self._buf = None
        self._len = 0
        self._need_ack = 0
        if self.__class__._next_seq_id == RM_SDK_LAST_SEQ_ID:
            self.__class__._next_seq_id = RM_SDK_FIRST_SEQ_ID
        else:
            self.__class__._next_seq_id += 1
        self._seq_id = self._next_seq_id
        self._proto = proto

    def __repr__(self):
        return "<{0}, {1}>".format(self.__class__.__name__, self._proto.resp)

    def pack(self):
        if self._proto:
            data_buf = self._proto.pack_req()
        """pack the proto to msg"""
        self._buf = data_buf
        return self._buf

    def unpack_protocol(self):
        self._proto = TextProtoDrone()
        if not self._proto.unpack_resp(self._buf):
            logger.warining("TextMsg: unpack_protocol, msg:{0}".format(self))
            return False
        return True

    def get_proto(self):
        return self._proto

    def get_buf(self):
        return self._buf

def decode_msg(buff, protocol="v1"):
    if protocol == "v1":
        if len(buff) < 4:
            logger.info("decode_msg, recv buf is not enouph.")
            return None, buff
        if buff[0] != 0x55:
            logger.warning("decode_msg, magic number is invalid.")
            return None, buff
        if algo.crc8_calc(buff[0:3]) != buff[3]:
            logger.warning("decode_msg, crc header check failed.")
            return None, buff
        msg_len = (buff[2] & 0x3) * 256 + buff[1]
        if len(buff) < msg_len:
            logger.warning("decode_msg, msg data is not enough, msg_len:{0}, buf_len:{1}".format(msg_len, len(buff)))
            return None, buff
        # unpack from byte array
        msg = Msg(buff[9], buff[10])
        msg._len = msg_len
        msg._seq_id = buff[7] * 256 + buff[6]
        msg._attri = buff[8]
        msg._sender = buff[4]
        msg._receiver = buff[5]
        msg._cmdset = int(buff[9])
        msg._cmdid = int(buff[10])
        msg._is_ack = msg._attri & 0x80 != 0
        msg._need_ack = (msg._attri & 0x60) >> 5
        msg._buf = buff[11:msg._len - 2]
        left_buf = buff[msg_len:]
        return msg, left_buf

    elif protocol == "text":
        # unpack
        msg = TextMsg()
        # filter out '\0xcc'
        if buff[0] == 204:
            logger.warning("decode_msg: recv invalid data, buff {0}".format(buff))
            return None, bytearray()
        else:
            msg._buf = buff.decode(encoding='utf-8')
            msg._len = len(msg._buf)
            return msg, bytearray()
        
class ProtoGetProductVersion(ProtoData):
    _cmdset = 0
    _cmdid = 0x4f
    _resp_size = 9

    def __init__(self):
        self._file_type = 4
        self._version = None

    def pack_req(self):
        buf = bytearray(self._resp_size)
        buf[0] = self._file_type
        buf[5] = 0xff
        buf[6] = 0xff
        buf[7] = 0xff
        buf[8] = 0xff
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            cc, bb, aa = struct.unpack_from("<HBB", buf, 9)
            self._version = "{0:02d}.{1:02d}.{2:04d}".format(aa, bb, cc)
            return True
        else:
            self._version = None
            logger.warning("ProtoGetProductVersion, unpack_resp, retcode {0}".format(self._retcode))
            return False

class ProtoGetVersion(ProtoData):
    _cmdset = 0
    _cmdid = 1
    _resp_size = 30

    def __init__(self):
        self._aa = 0
        self._bb = 1
        self._cc = 0
        self._dd = 0
        self._build = 1
        self._version = 0
        self._minor = 1
        self._major = 0
        self._cmds = 0
        self._rooback = 0
        self._retcode = 0

    def pack_req(self):
        return b''

    def unpack_resp(self, buf, offset=0):
        if len(buf) < self._resp_size:
            raise Exception("buf length is not enouph.")

        self._retcode = buf[0]
        if self._retcode != 0:
            return False
        self._aa = buf[0]
        self._bb = buf[1]
        self._cc = buf[2]
        self._dd = buf[3]
        return True

class ProtoGetSn(ProtoData):
    _cmdset = 0x0
    _cmdid = 0x51
    _req_size = 1

    def __init__(self):
        self._type = 1

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._type
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[offset]
        if self._retcode == 0:
            self._length = buf[offset + 1]
            self._sn = buf[offset + 3:self._length + offset + 3].decode('utf-8', 'ignore')
            return True
        else:
            return False

class ProtoSubscribeAddNode(ProtoData):
    _cmdset = 0x48
    _cmdid = 0x01
    _req_size = 5

    def __init__(self):
        self._node_id = 0
        self._sub_vision = 0x03000000
        self._pub_node_id = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        struct.pack_into("<BI", buf, 0, self._node_id, self._sub_vision)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0 or self._retcode == 0x50:
            self._pub_node_id = buf[1]
            return True
        else:
            logger.warning("ProtoSubscribeAddNode: unpack_resp, retcode:{0}".format(self._retcode))
            return False
        
class ProtoSetSdkConnection(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0xd4
    _req_size = 10

    def __init__(self):
        self._control = 0
        self._host = 0
        self._connection = 0
        self._protocol = 0
        self._ip = '0.0.0.0'
        self._port = 10010

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._control
        buf[1] = self._host
        buf[2] = self._connection
        buf[3] = self._protocol
        ip_bytes = bytes(map(int, self._ip.split('.')))
        buf[4:8] = ip_bytes
        struct.pack_into("<H", buf, 8, self._port)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            self._state = buf[1]
            if self._state == 2:
                self._config_ip = "{0:d}.{1:d}.{2:d}.{3:d}".format(buf[2], buf[3], buf[4], buf[5])
            return True
        else:
            return False

class ProtoSetRobotMode(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x46
    _req_size = 1

    def __init__(self):
        self._mode = 1

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._mode
        return buf

    def unpack_resp(self, buff, offset=0):
        self._retcode = buff[0]
        if self._retcode == 0:
            return True
        else:
            return False

class ProtoSubNodeReset(ProtoData):
    _cmdset = 0x48
    _cmdid = 0x02
    _req_size = 1

    def __init__(self):
        self._node_id = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._node_id
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False
        
class ProtoSdkHeartBeat(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0xd5
    _req_size = 0

    def __init__(self):
        pass

    def pack_req(self):
        return b''

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False

class ProtoSetSdkMode(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0xd1
    _req_size = 1

    def __init__(self):
        self._enable = 1

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._enable
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[offset]
        if self._retcode == 0:
            return True
        else:
            return False
        
class TextProtoData:
    SUCCESSFUL_RESP_FLAG = 'ok'

    def __init__(self):
        self._buf = None
        self._len = None
        self._text_cmd = None
        self._action_state = None
        self._resp = None
        self._percent = 0

    def __repr__(self):
        return "<{0}>".format(self.__class__.__name__)

    @property
    def text_cmd(self):
        return self._text_cmd

    @text_cmd.setter
    def text_cmd(self, cmd):
        self._text_cmd = cmd

    def pack_req(self):
        """ 协议对象打包发送数据为字节流。

        :return: 字节流数据。
        """
        logger.debug("TextProtoData: pack_req test_cmd {0}, type {1}".format(self.text_cmd, type(self.text_cmd)))
        self._buf = self.text_cmd
        return self._buf

    def unpack_req(self, buf, offset=0):
        """ 从字节流解包。

        :param buf：字节流数据。
        :param offset：字节流数据偏移量。
        :return：True 解包成功；False 解包失败。
        """
        self._action_state = buf
        self._resp = buf
        return True

    def pack_resp(self):
        """ 协议对象打包。

        :return：字节流数据。
        """
        pass

    def unpack_resp(self, buf, offset=0):
        """ 从字节流解包为返回值和相关属性。

        :param buf：字节流数据。
        :param offset：字节流数据偏移量。
        :return: True or False.
        """
        self._action_state = buf
        self._resp = buf
        return True

    def get_status(self):
        if self._resp:
            if self._resp == 'error':
                return False
            elif self._resp == 'ok':
                return True
            else:
                return False
        else:
            return False

    @property
    def resp(self):
        if self._resp is not None:
            return self._resp.strip()
        else:
            return self._resp

    @property
    def proresp(self):
        """ 针对acceleration?、attitude?、temp?命令的回复进行预处理。

        :return: dict.
        """
        msg_dict = dict()
        resp = self.resp

        if resp is None:
            return msg_dict

        if len(resp.split("~")) == 2:
            msg_dict["templ"] = int(resp.split("~")[0])
            msg_dict["temph"] = int(resp.split("~")[1][:-1])
        elif len(resp.split(";")) == 4:
            msg_list = resp.split(";")[:-1]
            for msg in msg_list:
                key, value = msg.split(":")
                msg_dict[key] = float(value)
        else:
            logger.warning("doesn't support sdk! proresp returns empty dict")
        return msg_dict


class TextProtoDrone(TextProtoData):

    def __init__(self):
        super().__init__()


class TextProtoDronePush(TextProtoData):
    def __init__(self):
        super().__init__()


class ProtoChassisSetWorkMode(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x19
    _req_size = 1

    def __init__(self):
        self._mode = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._mode
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False

class ProtoChassisStickOverlay(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x28
    _req_size = 1

    def __init__(self):
        self._mode = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._mode
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[offset]
        if self._retcode == 0:
            return True
        else:
            logger.warning("ProtoChassisStickOverlay: unpack_resp, retcode:{0}".format(self._retcode))
            return False
        

class ProtoSetWheelSpeed(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x20
    _req_size = 8

    def __init__(self):
        self._w1_spd = 0
        self._w2_spd = 0
        self._w3_spd = 0
        self._w4_spd = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        struct.pack_into("<hhhh", buf, 0, self._w1_spd, self._w2_spd, self._w3_spd, self._w4_spd)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            logger.warning("ProtoSetWheelSpeed: unpack_resp, retcode:{0}".format(self._retcode))
            return False

class ProtoChassisSpeedMode(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x21
    _req_size = 12
    _cmdtype = DUSS_MB_TYPE_PUSH

    def __init__(self):
        self._x_spd = float(0)
        self._y_spd = float(0)
        self._z_spd = float(0)

    def pack_req(self):
        buf = bytearray(self._req_size)
        struct.pack_into("<fff", buf, 0, self._x_spd, self._y_spd, self._z_spd)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False
        
class ProtoChassisPwmPercent(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x3c
    _req_size = 13
    _cmdtype = DUSS_MB_TYPE_REQ

    def __init__(self):
        self._mask = 0
        self._pwm1 = 0
        self._pwm2 = 0
        self._pwm3 = 0
        self._pwm4 = 0
        self._pwm5 = 0
        self._pwm6 = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._mask
        struct.pack_into('<HHHHHH', buf, 1, self._pwm1, self._pwm2, self._pwm3, self._pwm4, self._pwm5, self._pwm6)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False
        

class ProtoChassisPwmFreq(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x2b
    _req_size = 13
    _cmdtype = DUSS_MB_TYPE_REQ

    def __init__(self):
        self._mask = 0
        self._pwm1 = 0
        self._pwm2 = 0
        self._pwm3 = 0
        self._pwm4 = 0
        self._pwm5 = 0
        self._pwm6 = 0

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._mask
        struct.pack_into('<HHHHHH', buf, 1, self._pwm1, self._pwm2, self._pwm3, self._pwm4, self._pwm5, self._pwm6)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[0]
        if self._retcode == 0:
            return True
        else:
            return False

class ProtoPositionMove(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x25
    _req_size = 13

    def __init__(self):
        self._action_id = 0
        self._freq = 2
        self._action_ctrl = 0
        self._ctrl_mode = 0
        self._axis_mode = 0
        self._pos_x = 0
        self._pos_y = 0
        self._pos_z = 0
        self._vel_xy_max = 0
        self._agl_omg_max = 300

    def pack_req(self):
        buf = bytearray(self._req_size)
        buf[0] = self._action_id
        buf[1] = self._action_ctrl | self._freq << 2
        buf[2] = self._ctrl_mode
        buf[3] = self._axis_mode
        struct.pack_into('<hhh', buf, 4, self._pos_x, self._pos_y, self._pos_z)
        buf[10] = self._vel_xy_max
        struct.pack_into('<h', buf, 11, self._agl_omg_max)
        return buf

    def unpack_resp(self, buf, offset=0):
        self._retcode = buf[offset]
        if self._retcode == 0:
            self._accept = buf[offset + 1]
            return True
        else:
            logger.warning("ProtoPositionMove: unpack_resp, retcode:{0}".format(self._retcode))
            return False


class ProtoPositionPush(ProtoData):
    _cmdset = 0x3f
    _cmdid = 0x2a

    def __init__(self):
        self._action_id = 0
        self._percent = 0
        self._action_state = 0
        self._pos_x = 0
        self._pos_y = 0
        self._pos_z = 0

    def pack_req(self):
        return b''

    # ack push.
    def unpack_req(self, buf, offset=0):
        self._action_id = buf[0]
        self._percent = buf[1]
        self._action_state = buf[2]
        self._pos_x, self._pos_y, self._pos_z = struct.unpack_from('<hhh', buf, 3)
        return True

    def unpack_resp(self, buf, offset=0):
        self._action_id = buf[offset]
        self._percent = buf[offset + 1]
        self._action_state = buf[offset + 2]
        self._pos_x, self._pos_y, self._pos_z = struct.unpack_from('<hhh', buf, offset + 3)
        return True
    
class ProtoPushPeriodMsg(ProtoData):
    _cmdset = 0x48
    _cmdid = 0x8
    _type = DUSS_MB_TYPE_PUSH

    def __init__(self):
        self._sub_mode = 0
        self._msg_id = 0
        self._data_buf = None

    def pack_req(self):
        return b''

    def unpack_req(self, buf, offset=0):
        self._sub_mode = buf[0]
        self._msg_id = buf[1]
        self._data_buf = buf[2:]
        return True