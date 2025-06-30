from abc import abstractmethod
from queue import Queue
import collections
import threading
from src.robomaster import module
from src.robomaster import protocol
from concurrent.futures import ThreadPoolExecutor

SDK_FIRST_DDS_ID = 20
SDK_LAST_DDS_ID = 225

DDS_POSITION = "position"

SUB_UID_MAP = {
    DDS_POSITION: 0x00020009eeb7cece,
}

DDS_SUB_TYPE_PERIOD = 0

registered_subjects = {}

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

class Subcriber(module.Module):
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

    
        