from src.robomaster import protocol
from src.robomaster import logger

__all__ = ['Module']

# registered module dict
registered_modules = {} 

class _AutoRegisterModule(type):
    """ help to automatically register Proto Class where ever they're defined """

    def __new__(mcs, name, bases, attrs, **kw):
        return super().__new__(mcs, name, bases, attrs, **kw)

    def __init__(cls, name, bases, attrs, **kw):
        super().__init__(name, bases, attrs, **kw)
        key = attrs['_host']
        if key in registered_modules.keys():
            raise ValueError("Duplicate module class {0}".format(name))
        registered_modules[key] = cls

class Module(metaclass=_AutoRegisterModule):
    _host = 0
    _client = None
    _robot = None

    def __init__(self, robot):
        self._robot = robot
        self._client = robot.client

    @property
    def client(self):
        return self._client

    def reset(self):
        raise Exception("Module, reset function Not Implemented!")

    def start(self):
        pass

    def stop(self):
        pass

    def get_version(self):
        proto = protocol.ProtoGetVersion()
        msg = protocol.Msg(self.client.hostbyte, self._host, proto)
        try:
            resp_msg = self.client.send_sync_msg(msg)
            if resp_msg is not None:
                prot = resp_msg.get_proto()
                version = "{0:02d}.{1:02d}.{2:02d}.{3:02d}".format(prot._aa, prot._bb, prot._cc, prot._dd)
                return version
            else:
                logger.warning("Module: get_version, {0} failed.".format(self.__class__.__name__))
                return None
        except Exception as e:
            logger.warning("Module: get_version, {0} exception {1}.".format(self.__class__.__name__, str(e)))
            return None

    def _send_sync_proto(self, proto, target=None):
        if not self.client:
            return False

        if target:
            msg = protocol.Msg(self._client.hostbyte, target, proto)
        else:
            msg = protocol.Msg(self._client.hostbyte, self._host, proto)
        try:
            resp_msg = self._client.send_sync_msg(msg)
            if resp_msg:
                proto = resp_msg.get_proto()
                if proto._retcode == 0:
                    return True
                else:
                    logger.warning("{0}: send_sync_proto, proto:{1}, retcode:{2} ".format(self.__class__.__name__,
                                                                                          proto,
                                                                                          proto._retcode))
                    return False
            else:
                logger.warning("{0}: send_sync_proto, proto:{1} resp_msg is None.".format(
                    self.__class__.__name__, proto))
                return False
        except Exception as e:
            logger.warning("{0}: send_sync_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False

    def _send_async_proto(self, proto, target=None):
        if not self.client:
            return False

        if target:
            msg = protocol.Msg(self._client.hostbyte, target, proto)
        else:
            msg = protocol.Msg(self._client.hostbyte, self._host, proto)
        try:
            return self._client.send_async_msg(msg)
        except Exception as e:
            logger.error("{0}: _send_async_proto, proto:{1}, exception:{2}".format(self.__class__.__name__, proto, e))
            return False
