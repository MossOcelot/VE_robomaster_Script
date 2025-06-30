from abc import abstractmethod
from src.robomaster import protocol


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