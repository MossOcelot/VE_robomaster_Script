from abc import abstractmethod
from queue import Queue
import collections
import threading
from src.robomaster import logger
from src.robomaster import module
from src.robomaster import protocol
from concurrent.futures import ThreadPoolExecutor

SDK_FIRST_DDS_ID = 20
SDK_LAST_DDS_ID = 225

DDS_ATTITUDE = "attitude"
DDS_IMU = "imu"
DDS_POSITION = "position"
DDS_SA_STATUS = "sa_status"

SUB_UID_MAP = {
    DDS_ATTITUDE: 0x000200096b986306,
    DDS_POSITION: 0x00020009eeb7cece,
    DDS_SA_STATUS: 0x000200094a2c6d55,
    DDS_IMU: 0x00020009a7985b8d,
}

DDS_SUB_TYPE_EVENT = 1
DDS_SUB_TYPE_PERIOD = 0

registered_subjects = {}
dds_cmd_filter = {(0x48, 0x08)}


class _AutoRegisterSubject(type):
    '''hepler to automatically register Proto Class whereever they're defined '''

    def __new__(mcs, name, bases, attrs, **kw):
        return super().__new__(mcs, name, bases, attrs, **kw)

    def __init__(cls, name, bases, attrs, **kw):
        super().__init__(name, bases, attrs, **kw)
        if name == 'Subject':
            return
        key = name
        if key in registered_subjects.keys():
            raise ValueError("Duplicate Subject class {0}".format(name))
        registered_subjects[key] = cls

class Subject(metaclass=_AutoRegisterSubject):
    name = "Subject"
    _push_proto_cls = protocol.ProtoPushPeriodMsg
    type = DDS_SUB_TYPE_PERIOD
    uid = 0
    freq = 1

    def __init__(self):
        self._task = None
        self._subject_id = 1
        self._callback = None
        self._cb_args = None
        self._cb_kw = None

    def __repr__(self):
        return "dds subject, name:{0}".format(self.name)

    def set_callback(self, callback, args, kw):
        self._callback = callback
        self._cb_args = args
        self._cb_kw = kw

    @abstractmethod
    def data_info(self):
        return None

    def exec(self):
        self._callback(self.data_info(), *self._cb_args, **self._cb_kw)

class SubHandler(collections.namedtuple("SubHandler", ("obj subject f"))):
    __slots__ = ()

class Subscriber(module.Module):
    _host = protocol.host2byte(9, 0)
    _sub_msg_id = SDK_FIRST_DDS_ID

    def __init__(self, robot):
        super().__init__(robot)
        self._robot = robot
        
        self.msg_sub_dict = {}
        self._publisher = collections.defaultdict(list)
        self._msg_queue = Queue()
        self._dispatcher_running = False
        self._dispatcher_thread = None
        self.excutor = ThreadPoolExecutor(max_workers=15)

    def __del__(self):
        self.stop()

    def get_next_subject_id(self):
        if self._sub_msg_id > SDK_LAST_DDS_ID:
            self._sub_msg_id = SDK_FIRST_DDS_ID
        else:
            self._sub_msg_id += 1
        return self._sub_msg_id

    def start(self):
        self._dds_mutex = threading.Lock()
        self._client.add_handler(self, "Subscriber", self._msg_recv)
        self._dispatcher_thread = threading.Thread(target=self._dispatch_task)
        self._dispatcher_thread.start()

    def stop(self):
        self._dispatcher_running = False
        if self._dispatcher_thread:
            self._msg_queue.put(None)
            self._dispatcher_thread.join()
            self._dispatcher_thread = None
        self.excutor.shutdown(wait=False)

    @classmethod
    def _msg_recv(cls, self, msg):
        for cmd_set, cmd_id in list(dds_cmd_filter):
            if msg.cmdset == cmd_set and msg.cmdid == cmd_id:
                 self._msg_queue.put(msg)

    def _dispatch_task(self):
        self._dispatcher_running = True
        logger.info("Subscriber: dispatcher_task is running...")
        while self._dispatcher_running:
            msg = self._msg_queue.get(1)
            if msg is None:
                if not self._dispatcher_running:
                    break
                continue
            self._dds_mutex.acquire()
            for name in self._publisher:
                handler = self._publisher[name]
                logger.debug("Subscriber: msg: {0}".format(msg))
                proto = msg.get_proto()
                if proto is None:
                    logger.warning("Subscriber: _publish, msg.get_proto None, msg:{0}".format(msg))
                    continue
                # logger.debug("handler type: {0}, DDS_SUB_TYPE_PERIOD: {1}".format(handler.subject.type, DDS_SUB_TYPE_PERIOD))
                if handler.subject.type == DDS_SUB_TYPE_PERIOD and\
                        msg.cmdset == 0x48 and msg.cmdid == 0x08:
                    logger.debug("Subscriber: _publish: msg_id:{0}, subject_id:{1}".format(proto._msg_id,
                                                                                           handler.subject._subject_id))
                    if proto._msg_id == handler.subject._subject_id:
                        logger.debug("test test test")
                        handler.subject.decode(proto._data_buf)
                        logger.debug("close close")
                        if handler.subject._task is None:
                            handler.subject._task = self.excutor.submit(handler.subject.exec)
                        if handler.subject._task.done() is True:
                            handler.subject._task = self.excutor.submit(handler.subject.exec)
                elif handler.subject.type == DDS_SUB_TYPE_EVENT:
                    if handler.subject.cmdset == msg.cmdset and handler.subject.cmdid == msg.cmdid:
                        handler.subject.decode(proto._data_buf)
                        if handler.subject._task is None:
                            handler.subject._task = self.excutor.submit(handler.subject.exec)
                        if handler.subject._task.done() is True:
                            handler.subject._task = self.excutor.submit(handler.subject.exec)
            self._dds_mutex.release()
            logger.info("Subscriber: _publish, msg is {0}".format(msg))

    def add_cmd_filter(self, cmd_set, cmd_id):
        dds_cmd_filter.add((cmd_set, cmd_id))

    def del_cmd_filter(self, cmd_set, cmd_id):
        dds_cmd_filter.remove((cmd_set, cmd_id))

    def add_subject_event_info(self, subject, callback=None, *args):
        """
        Add an event-based subscription

        :param subject: The subject instance corresponding to the event
        :param callback: The function to handle or parse the event data
        """
        # For event subscription, only a filter is added (no periodic task)
        subject.set_callback(callback, args[0], args[1])
        handler = SubHandler(self, subject, callback)
        subject._task = None
        self._dds_mutex.acquire()
        self._publisher[subject.name] = handler
        self._dds_mutex.release()
        self.add_cmd_filter(subject.cmdset, subject.cmdid)
        logger.debug("Subscriber: add_subject_event_info, subject:{0}, cmdset:{1}, cmdid:{2}".format(
            subject.name, subject.cmdset, subject.cmdid))
        return True
    
    def del_subject_event_info(self, subject):
        """
        Remove an event-based subscription

        :param subject: The subject instance corresponding to the event
        :return: bool: Result of the operation
        """
        # For event subscriptions, only remove the filter (no task cancellation needed unless one exists)
        if self._publisher[subject.name].subject._task is None:
            pass
        elif self._publisher[subject.name].subject._task.done() is False:
            self._publisher[subject.name].subject._task.cancel()
        self.del_cmd_filter(subject.cmdset, subject.cmdid)
        return True

    def add_subject_info(self, subject, callback=None, *args):
        """
        Request to subscribe to data (low-level interface)

        :param subject: The subject instance representing the data subscription
        :param callback: The function used to parse or handle the incoming subscribed data
        :return: bool: Result of the request (True if successful)
        """
        # Add the subscription handler to the publisher registry
        subject.set_callback(callback, args[0], args[1])
        handler = SubHandler(self, subject, callback)

        self._dds_mutex.acquire()
        self._publisher[subject.name] = handler
        self._dds_mutex.release()

        logger.debug(f"publisher: {self._publisher.keys()}")

        # Construct and send a protocol message to initiate the subscription
        proto = protocol.ProtoAddSubMsg()
        proto._node_id = self.client.hostbyte
        proto._sub_freq = subject.freq
        proto._sub_data_num = 1
        proto._msg_id = self.get_next_subject_id()
        subject._subject_id = proto._msg_id
        subject._task = None
        proto._sub_uid_list.append(subject.uid)
        logger.debug("subscriber")
        return self._send_sync_proto(proto, protocol.host2byte(9, 0))

    def del_subject_info(self, subject_name):
        """
        Delete a data subscription

        :param subject_name: The name of the subject to unsubscribe from
        :return: bool: Result of the unsubscription operation
        """
        logger.debug("Subscriber: del_subject_info: name:{0}, self._publisher:{1}".format(subject_name,
                     self._publisher))
        if subject_name in self._publisher:
            subject_id = self._publisher[subject_name].subject._subject_id
            if self._publisher[subject_name].subject._task.done() is False:
                self._publisher[subject_name].subject._task.cancel()
            self._dds_mutex.acquire()
            del self._publisher[subject_name]
            self._dds_mutex.release()
            proto = protocol.ProtoDelMsg()
            proto._msg_id = subject_id
            proto._node_id = self.client.hostbyte
            return self._send_sync_proto(proto, protocol.host2byte(9, 0))
        else:
            logger.warning("Subscriber: fail to del_subject_info", subject_name)