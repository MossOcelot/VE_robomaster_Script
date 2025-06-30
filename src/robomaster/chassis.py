import struct
from src.robomaster import dds
from src.robomaster import logger
from src.robomaster import module
from src.robomaster import protocol
from src.robomaster import util
from src.robomaster import action
import threading

__all__ = ['Chassis', 'ChassisMoveAction']

class ChassisMoveAction(action.Action):
    _action_proto_cls = protocol.ProtoPositionMove
    _push_proto_cls = protocol.ProtoPositionPush
    _target = protocol.host2byte(3, 6)

    def __init__(self, x=0, y=0, z=0, spd_xy=0, spd_z=0, **kw):
        super().__init__(**kw)
        self._x = x
        self._y = y
        self._z = z
        self._spd_xy = spd_xy
        self._spd_z = spd_z

    def __repr__(self):
        return "action_id:{0}, state:{1}, percent:{2}, x:{3}, y:{4}, z:{5}, xy_speed:{6}, z_speed:{7}".format(
            self._action_id, self._state, self._percent, self._x, self._y, self._z, self._spd_xy, self._spd_z)

    def encode(self):
        proto = protocol.ProtoPositionMove()
        proto._pos_x = util.CHASSIS_POS_X_SET_CHECKER.val2proto(self._x)
        proto._pos_y = util.CHASSIS_POS_Y_SET_CHECKER.val2proto(self._y)
        proto._pos_z = util.CHASSIS_POS_Z_SET_CHECKER.val2proto(self._z)
        # The spd_xy limit to [0.5, 2.0]
        if self._spd_xy < 0.5:
            self._spd_xy = 0.5
            logger.warning("spd_xy: below limit and is set to 0.5")
        if self._spd_xy > 2.0:
            self._spd_xy = 2.0
            logger.warning("spd_xy: over limit and is set to 2.0")
        proto._vel_xy_max = int(160 * self._spd_xy - 70)
        # The spd_z limit to [10, 540]
        if self._spd_z < 10:
            self._spd_z = 10
            logger.warning("spd_z: below limit and is set to 10")
        if self._spd_z > 540:
            self._spd_z = 540
            logger.warning("spd_z: over limit and is set to 540")
        proto._agl_omg_max = int(self._spd_z * 10)
        return proto

    def update_from_push(self, proto):
        if proto.__class__ is not self._push_proto_cls:
            return

        self._percent = proto._percent
        self._update_action_state(proto._action_state)

        self._pos_x = util.CHASSIS_POS_X_SET_CHECKER.proto2val(proto._pos_x)
        self._pos_y = util.CHASSIS_POS_Y_SET_CHECKER.proto2val(proto._pos_y)
        self._pos_z = util.CHASSIS_POS_Z_SET_CHECKER.proto2val(proto._pos_z)
        logger.info("{0} update_from_push: {1}".format(self.__class__.__name__, self))

class PositionSubject(dds.Subject):
    name = dds.DDS_POSITION
    uid = dds.SUB_UID_MAP[name]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self, cs):
        self._position_x = 0
        self._position_y = 0
        self._position_z = 0
        self._cs = cs
        self._offset_x = 0
        self._offset_y = 0
        self._offset_z = 0
        self._first_flag = True

    def position(self):
        return self._position_x, self._position_y, self._position_z

    def data_info(self):
        ''' If cs=0, use current position as coordinate origin;
            otherwise, use the robot’s initial (power-on) position as origin. '''
        if self._cs == 0:
            if self._first_flag:
                self._offset_x = self._position_x
                self._offset_y = self._position_y
                self._offset_z = self._position_z
                self._first_flag = False
            self._position_x = self._position_x - self._offset_x
            self._position_y = self._position_y - self._offset_y
            self._position_z = self._position_z - self._offset_z
        self._position_x = util.CHASSIS_POS_X_SUB_CHECKER.proto2val(self._position_x)
        self._position_y = util.CHASSIS_POS_Y_SUB_CHECKER.proto2val(self._position_y)
        self._position_z = util.CHASSIS_POS_Z_SUB_CHECKER.proto2val(self._position_z)
        return self._position_x, self._position_y, self._position_z

    def decode(self, buf):
        self._position_x, self._position_y, self._position_z = struct.unpack('<fff', buf)


class Chassis(module.Module):
    _host = protocol.host2byte(3, 6)

    def __init__(self, robot):
        super().__init__(robot)
        self._action_dispatcher = robot.action_dispatcher
        self._auto_timer = None

    def stop(self):
        if self._auto_timer:
            if self._auto_timer.is_alive():
                self._auto_timer.cancel()
        super().stop()

    def _set_mode(self, mode):
        proto = protocol.ProtoChassisSetWorkMode()
        return self._send_sync_proto(proto)

    def _get_mode(self):
        proto = protocol.ProtoChassisGetWorkMode()
        return self._send_sync_proto(proto)
    
    def stick_overlay(self, fusion_mode=0):
        proto = protocol.ProtoChassisStickOverlay()
        proto._mode = fusion_mode
        return self._send_sync_proto(proto)
    
    def drive_wheels(self, w1=0, w2=0, w3=0, w4=0, timeout=None):
        """
            Set the Mecanum wheel speeds

            :param w1: int: [-1000, 1000], front-right wheel speed, positive for forward rotation in the direction of the robot's front, unit: rpm
            :param w2: int: [-1000, 1000], front-left wheel speed, positive for forward rotation in the direction of the robot's front, unit: rpm
            :param w3: int: [-1000, 1000], rear-left wheel speed, positive for forward rotation in the direction of the robot's front, unit: rpm
            :param w4: int: [-1000, 1000], rear-right wheel speed, positive for forward rotation in the direction of the robot's front, unit: rpm
            :param timeout: float: (0, ∞), if no wheel speed command is received within the specified time, the robot will automatically stop, unit: seconds
        """
        proto = protocol.ProtoSetWheelSpeed()
        proto._w1_spd = util.WHEEL_SPD_CHECKER.val2proto(w1)
        proto._w2_spd = util.WHEEL_SPD_CHECKER.val2proto(-w2)
        proto._w3_spd = util.WHEEL_SPD_CHECKER.val2proto(-w3)
        proto._w4_spd = util.WHEEL_SPD_CHECKER.val2proto(w4)
        if timeout:
            if self._auto_timer:
                if self._auto_timer.is_alive():
                    self._auto_timer.cancel()
            self._auto_timer = threading.Timer(timeout, self._auto_stop_timer, args=("drive_wheels",))
            self._auto_timer.start()
            return self._send_sync_proto(proto)
        return self._send_sync_proto(proto)
    
    def _auto_stop_timer(self, api="drive_speed"):
        if api == "drive_speed":
            logger.info("Chassis: drive_speed timeout, auto stop!")
            self.drive_speed(0, 0, 0, 0)
        elif api == "drive_wheels":
            logger.info("Chassis: drive_wheels timeout, auto stop!")
            self.drive_wheels(0, 0, 0, 0)
        else:
            logger.warning("Chassis: unsupported api:{0}".format(api))

    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        """
            Set chassis velocity, effective immediately

            :param x: float: [-3.5, 3.5], velocity along the x-axis (forward/backward), unit: m/s
            :param y: float: [-3.5, 3.5], velocity along the y-axis (sideways), unit: m/s
            :param z: float: [-600, 600], rotational velocity around the z-axis, unit: °/s
            :param timeout: float: (0, ∞), if no velocity command is received within the specified time, the robot will automatically stop, unit: seconds
        """
        proto = protocol.ProtoChassisSpeedMode()
        proto._x_spd = util.CHASSIS_SPD_X_CHECKER.val2proto(x)
        proto._y_spd = util.CHASSIS_SPD_Y_CHECKER.val2proto(y)
        proto._z_spd = util.CHASSIS_SPD_Z_CHECKER.val2proto(z)
        logger.info("x_spd:{0:f}, y_spd:{1:f}, z_spd:{2:f}".format(proto._x_spd, proto._y_spd, proto._z_spd))
        if timeout:
            if self._auto_timer:
                if self._auto_timer.is_alive():
                    self._auto_timer.cancel()
            self._auto_timer = threading.Timer(timeout, self._auto_stop_timer, args=("drive_speed",))
            self._auto_timer.start()
            return self._send_sync_proto(proto)
        return self._send_sync_proto(proto)
    
    def set_pwm_value(self, pwm1=None, pwm2=None, pwm3=None, pwm4=None, pwm5=None, pwm6=None):
        """
            Set PWM output duty cycle

            :param pwm1: int: [0, 100], PWM output duty cycle, unit: %
            :param pwm2: int: [0, 100], PWM output duty cycle, unit: %
            :param pwm3: int: [0, 100], PWM output duty cycle, unit: %
            :param pwm4: int: [0, 100], PWM output duty cycle, unit: %
            :param pwm5: int: [0, 100], PWM output duty cycle, unit: %
            :param pwm6: int: [0, 100], PWM output duty cycle, unit: %
        """
        proto = protocol.ProtoChassisPwmPercent()
        proto._mask = 0
        if pwm1:
            proto._mask = 1
            proto._pwm1 = util.PWM_VALUE_CHECKER.val2proto(pwm1)
        if pwm2:
            proto._mask |= (1 << 1)
            proto._pwm2 = util.PWM_VALUE_CHECKER.val2proto(pwm2)
        if pwm3:
            proto._mask |= (1 << 2)
            proto._pwm3 = util.PWM_VALUE_CHECKER.val2proto(pwm3)
        if pwm4:
            proto._mask |= (1 << 3)
            proto._pwm4 = util.PWM_VALUE_CHECKER.val2proto(pwm4)
        if pwm5:
            proto._mask |= (1 << 4)
            proto._pwm5 = util.PWM_VALUE_CHECKER.val2proto(pwm5)
        if pwm6:
            proto._mask |= (1 << 5)
            proto._pwm6 = util.PWM_VALUE_CHECKER.val2proto(pwm6)
        return self._send_sync_proto(proto)
    
    def set_pwm_freq(self, pwm1=None, pwm2=None, pwm3=None, pwm4=None, pwm5=None, pwm6=None):
        """
            Set PWM output frequency

            :param pwm1~6: int: [0, 50000], PWM output frequency, unit: Hz
        """
        proto = protocol.ProtoChassisPwmFreq()
        proto._mask = 0
        if pwm1:
            proto._mask = 1
            proto._pwm1 = util.PWM_VALUE_CHECKER.val2proto(pwm1)
        if pwm2:
            proto._mask |= (1 << 1)
            proto._pwm2 = util.PWM_VALUE_CHECKER.val2proto(pwm2)
        if pwm3:
            proto._mask |= (1 << 2)
            proto._pwm3 = util.PWM_VALUE_CHECKER.val2proto(pwm3)
        if pwm4:
            proto._mask |= (1 << 3)
            proto._pwm4 = util.PWM_VALUE_CHECKER.val2proto(pwm4)
        if pwm5:
            proto._mask |= (1 << 4)
            proto._pwm5 = util.PWM_VALUE_CHECKER.val2proto(pwm5)
        if pwm6:
            proto._mask |= (1 << 5)
            proto._pwm6 = util.PWM_VALUE_CHECKER.val2proto(pwm6)
        return self._send_sync_proto(proto)

    
    # actions.
    def move(self, x=0, y=0, z=0, xy_speed=0.5, z_speed=30):
        """
            Control chassis movement to a specified position, with the current position as the origin

            :param x: float: [-5, 5], movement distance along the x-axis, unit: meters
            :param y: float: [-5, 5], movement distance along the y-axis, unit: meters
            :param z: float: [-1800, 1800], rotation angle around the z-axis, unit: degrees
            :param xy_speed: float: [0.5, 2], movement speed along the x and y axes, unit: m/s
            :param z_speed: float: [10, 540], rotational speed around the z-axis, unit: °/s
            :return: returns an action object
        """
        action = ChassisMoveAction(x, y, z, xy_speed, z_speed)
        self._action_dispatcher.send_action(action)
        return action
    
    # Data Subscription Interface
    # def sub_position(self, cs=0, freq=5, callback=None, *args, **kw):
    #     """ Subscribe to chassis position data

    #     :param cs: int: [0, 1] Coordinate system selection for chassis position:
    #                     0 = relative to current robot position
    #                     1 = relative to initial (power-on) position

    #     :param freq: enum: (1, 5, 10, 20, 50) Frequency of data updates, in Hz

    #     :param callback: Function to be called when data is received. Returns a tuple (x, y, z):
    #                     :x: Distance along the x-axis, in meters
    #                     :y: Distance along the y-axis, in meters
    #                     :z: Distance along the z-axis (rotation or height depending on context), in meters

    #     :param args: Additional arguments for the callback
    #     :param kw: Keyword arguments for the callback
    #     :return: bool: Whether the subscription was successfully registered
    #     """
    #     sub = self._robot.dds # property dds on robot
    #     subject = PositionSubject(cs)
    #     subject.freq = freq
    #     return sub.add_subject_info(subject, callback, args, kw)
