
from odrive import enums
import rospy


class OdriveMotor(object):
    """Base class to communicate with a specific motor connected to a specified Odrive."""

    # The states are used to set the specified axis to a designated operating state, look at odrive enums for more info
    AXIS_STATES = {axis_state: nr for (axis_state, nr) in enums.__dict__.items() if axis_state.startswith('AXIS')}

    # The modes are used to set important operating parameters, look at odrive enums for more info
    MOTOR_MODES = {mode_name: nr for (mode_name, nr) in enums.__dict__.items() if mode_name.startswith('MOTOR')}
    CONTROL_MODES = {mode_name: nr for (mode_name, nr) in enums.__dict__.items() if mode_name.startswith('CTRL')}
    ENCODER_MODES = {mode_name: nr for (mode_name, nr) in enums.__dict__.items() if mode_name.startswith('AXIS')}

    # The error codes are used to translate an error code to an error message, look at the odrive enums for more info
    AXIS_ERRORS = {name: nr for (name, nr) in enums.errors.axis.__dict__.items() if name.startswith('ERROR')}
    MOTOR_ERRORS = {name: nr for (name, nr) in enums.errors.motor.__dict__.items() if name.startswith('ERROR')}
    ENCODER_ERRORS = {name: nr for (name, nr) in enums.errors.encoder.__dict__.items() if name.startswith('ERROR')}
    CONTROL_ERRORS = {name: nr for (name, nr) in enums.errors.controller.__dict__.items() if name.startswith('ERROR')}

    # A standard error message used when given type does not match the requested type
    TYPE_ERROR_MSG = 'Given type {gt} for {nm} is insufficient, requested type is {rt}'

    def __init__(self, odrive, odrive_axis):
        """Initialise the odrive motor object to communicate with the odrive and specific axis.

        :param odrive: an odrive object (example: odrv0)
        :param odrive_axis: specify the axis of the odrive which represent the connected motor
        """
        self._odrive = odrive
        self._axis = odrive_axis

    @classmethod
    def create_odrive(cls, odrive_connection_manager, serial_number, axis_nr):
        """Create an Odrive object to communicate with a specific motor.

        :param odrive_connection_manager: The object which holds all the connections to the odrives
        :param serial_number: Serial number reference of the odrive in hex
        :param axis_nr: The axis number of the motor ['axis0', 'axis1']
        """
        odrive = odrive_connection_manager[serial_number]

        if odrive is None:
            rospy.logerr('Odrive with serial number: {sn}, could not be found'.format(sn=serial_number))
            return None

        axis_reference = getattr(odrive, axis_nr, None)

        if axis_reference is None:
            rospy.logerr('Odrive {sn} does not have axis: {ax}'.format(sn=serial_number, ax=axis_nr))
            return None

        return cls(odrive, axis_reference)

    # region Odrive variables
    @property
    def voltage(self):
        """Return the voltage on the odrive."""
        return self._odrive.vbus_voltage

    @property
    def serial_number(self):
        """Return the serial number linked to this odrive."""
        return '{sn:12X}'.format(sn=self._odrive.serial_number)

    @property
    def uptime(self):
        """Return the amount of seconds the odrive is active."""
        return self._odrive.uptime

    @property
    def general_error(self):
        """Return the error state of the axis."""
        return self._axis.error

    def set_configuration(self):
        """Save the current configurations to the odrive."""
        self._odrive.save_configuration()

    def erase_configuration(self):
        """Delete the configuration on the odrive."""
        self._odrive.erase_configuration()

    def reboot(self):
        """Reboot the odrive."""
        self._odrive.reboot()

    @property
    def brake_resistance(self):
        """Return the odrive brake resistance value."""
        return self._odrive.config.brake_resistance

    @brake_resistance.setter
    def brake_resistance(self, brake_resistance):
        """Set a new value as brake resistance.

        :param brake_resistance: A new brake resistance value as float
        """
        if not isinstance(brake_resistance, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(brake_resistance), nm='brake_resistance', rt='float'))
        self._odrive.config.brake_resistance = brake_resistance
    # endregion

    # region Axis variables
    def clear_errors(self):
        """Manually clear all the axis errors."""
        self._axis.handle.error = 0
        self._axis.handle.encoder.error = 0
        self._axis.handle.motor.error = 0
        self._axis.handle.sensorless_estimator.error = 0
    # endregion

    # region Motor variables
    @property
    def motor_error(self):
        """Return the axis specific error."""
        return self._axis.motor.error

    @property
    def is_calibrated(self):
        """Return if the motor has successfully calibrated."""
        return self._axis.motor.is_calibrated

    @property
    def pre_calibrated(self):
        """Return the pre calibrated value."""
        return self._axis.motor.config.pre_calibrated

    @pre_calibrated.setter
    def pre_calibrated(self, pre_calibrated_val):
        """Set the pre calibrated value to the specified axis.

        :param pre_calibrated_val: The pre calibration value as bool
        """
        if not isinstance(pre_calibrated_val, int):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(pre_calibrated_val), nm='pre_calibrated_val', rt='int'))
        self._axis.motor.config.pre_calibrated = pre_calibrated_val

    @property
    def state(self):
        """Return the current state of the motor."""
        return self._axis.current_state

    @state.setter
    def state(self, requested_state):
        """Set a new requested state to the specified axis on the odrive.

        :param requested_state: a new requested state (see odrive enums)
        """
        if not isinstance(requested_state, int):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(requested_state), nm='requested_state', rt='int'))
        self._axis.requested_state = requested_state

    @property
    def watchdog_timer(self):
        """Return the currently set watchdog timer."""
        return self._axis.config.watchdog_timeout

    @watchdog_timer.setter
    def watchdog_timer(self, watchdog_timeout):
        """Set the watchdog timer using the watchdog property.

        :param watchdog_timeout: the watchdog time as float
        """
        if not isinstance(watchdog_timeout, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(watchdog_timeout), nm='watchdog_timeout', rt='float'))
        self._axis.config.watchdog_timeout = watchdog_timeout

    @property
    def pole_pairs(self):
        """Return the amount of pole pairs of the motor."""
        return self._axis.motor.config.pole_pairs

    @pole_pairs.setter
    def pole_pairs(self, pole_pairs):
        """Set the amount of pole pairs of the attached motor.

        :param pole_pairs: The amount of pole pairs as int32
        """
        if not isinstance(pole_pairs, int):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(pole_pairs), nm='pole_pairs', rt='int32'))
        self._axis.config.pole_pairs = pole_pairs

    @property
    def phase_resistance(self):
        """Return the phase resistance of the motor."""
        return self._axis.motor.config.phase_resistance

    @phase_resistance.setter
    def phase_resistance(self, phase_resistance):
        """Set a phase resistance corresponding to the motor.

        :param phase_resistance: The phase resistance as float
        """
        if not isinstance(phase_resistance, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(phase_resistance), nm='phase_resistance', rt='float'))
        self._axis.motor.config.phase_resistance = phase_resistance

    @property
    def motor_type(self):
        """Return the motor type (use the enums of the odrive library to translate return value)."""
        return self._axis.motor.config.motor_type

    @motor_type.setter
    def motor_type(self, motor_type):
        """Set a motor type to the odrive (use the enums of the odrive library as reference values).

        :param motor_type: The motor type
        """
        if not isinstance(motor_type, int) or motor_type not in (0, 1, 2):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(motor_type), nm='motor_type', rt='int (0, 1, 2)'))
        self._axis.motor.config.motor_type = motor_type

    @property
    def direction(self):
        """Return the motor direction."""
        return self._axis.motor.config.direction

    @direction.setter
    def direction(self, motor_direction):
        """Set the new motor direction to the specified axis.

        :param motor_direction: The motor direction as int (0, 1)
        """
        if not isinstance(motor_direction, int) or motor_direction not in (0, 1):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(motor_direction), nm='motor_direction', rt='int (0, 1)'))
        self._axis.motor.config.direction = motor_direction
    # endregion

    # region Controller variables
    @property
    def control_error(self):
        """Return the controller specific error."""
        return self._axis.controller.error

    @property
    def control_mode(self):
        """Return the control mode of the motor."""
        return self._axis.controller.config.control_mode

    @control_mode.setter
    def control_mode(self, ctrl_mode):
        """Set the control mode of the motor using the control mode property.

        :param ctrl_mode: The control mode as int8
        """
        if not isinstance(ctrl_mode, int) or ctrl_mode not in (0, 1, 2, 3, 4):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(ctrl_mode), nm='ctrl_mode', rt='int8 (0, 1, 2, 3, 4)'))
        self._axis.controller.config.control_mode = ctrl_mode

    @property
    def current_limit(self):
        """Return the current limit of the motor."""
        return self._axis.motor.config.current_lim

    @current_limit.setter
    def current_limit(self, current_limit):
        """Set the current limit using the current limit property.

        :param current_limit: The current limit as float
        """
        if not isinstance(current_limit, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(current_limit), nm='current_limit', rt='float'))
        self._axis.motor.config.current_lim = current_limit

    @property
    def position_gain(self):
        """Return the position gain of the controller."""
        return self._axis.controller.config.pos_gain

    @position_gain.setter
    def position_gain(self, new_pos_gain):
        """Set a new position gain value to the controller.

        :param new_pos_gain: a new position gain as float
        """
        if not isinstance(new_pos_gain, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(new_pos_gain), nm='new_pos_gain', rt='float'))
        self._axis.controller.config.pos_gain = new_pos_gain

    @property
    def velocity_gain(self):
        """Return the velocity gain of the controller."""
        return self._axis.controller.config.vel_gain

    @velocity_gain.setter
    def velocity_gain(self, new_vel_gain):
        """Set a new velocity gain value to the controller.

        :param new_vel_gain: a new velocity gain as float
        """
        if not isinstance(new_vel_gain, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(new_vel_gain), nm='new_vel_gain', rt='float'))
        self._axis.controller.config.vel_gain = new_vel_gain

    @property
    def velocity_limit(self):
        """Return the velocity limit of the controller."""
        return self._axis.controller.config.vel_limit

    @velocity_limit.setter
    def velocity_limit(self, new_vel_limit):
        """Set a new velocity limit to the controller.

        :param new_vel_limit: a new velocity limit as float
        """
        if not isinstance(new_vel_limit, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(new_vel_limit), nm='new_vel_limit', rt='float'))
        self._axis.controller.config.vel_limit = new_vel_limit

    @property
    def velocity_integrator_gain(self):
        """Return the velocity integrator gain value."""
        return self._axis.controller.config.vel_integrator_gain

    @velocity_integrator_gain.setter
    def velocity_integrator_gain(self, vel_integrator_g):
        """Set a new value for the velocity integrator gain.

        :param vel_integrator_g: The new velocity integrator gain as float.
        """
        if not isinstance(vel_integrator_g, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(vel_integrator_g), nm='vel_integrator_g', rt='float'))
        self._axis.controller.config.vel_integrator_gain = vel_integrator_g
    # endregion

    # region Actuating variables
    @property
    def position_setpoint(self):
        """Return the position setpoint (encoder counts) of the controller."""
        return self._axis.controller.pos_setpoint

    @position_setpoint.setter
    def position_setpoint(self, new_pos_setpoint):
        """Set a new position setpoint to the axis.

        :param new_pos_setpoint: A new position (encoder counts) setpoint as float
        """
        if not isinstance(new_pos_setpoint, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(new_pos_setpoint), nm='new_pos_setpoint', rt='float'))
        self._axis.controller.set_pos_setpoint(new_pos_setpoint)

    @property
    def velocity_setpoint(self):
        """Return the velocity setpoint (encoder counts/s) of the controller."""
        return self._axis.controller.vel_setpoint

    @velocity_setpoint.setter
    def velocity_setpoint(self, new_vel_setpoint):
        """Set a new velocity setpoint to the axis.

        :param new_vel_setpoint: A new velocity setpoint (encoder counts/s) as float
        """
        if not isinstance(new_vel_setpoint, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(new_vel_setpoint), nm='new_vel_setpoint', rt='float'))
        self._axis.controller.set_vel_setpoint(new_vel_setpoint)

    @property
    def current_setpoint(self):
        """Return the current setpoint [A] of the controller."""
        return self._axis.controller.current_setpoint

    @current_setpoint.setter
    def current_setpoint(self, current_setpoint):
        """Set a new current setpoint to the axis.

        :param current_setpoint: A new current setpoint [A] as float
        """
        if not isinstance(current_setpoint, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(current_setpoint), nm='current_setpoint', rt='float'))
        self._axis.controller.set_current_setpoint(current_setpoint)
    # endregion

    # region Encoder variables
    @property
    def encoder_error(self):
        """Return the error state of the encoder."""
        return self._axis.encoder.error

    @property
    def encoder_ready(self):
        """Return the boolean if the encoder is ready."""
        return self._axis.encoder.is_ready

    @property
    def encoder_index_found(self):
        """Return the boolean if the encoder has found an index."""
        return self._axis.encoder.index_found

    @property
    def encoder_counts_in_cpr(self):
        """Return the counts per revolution."""
        return self._axis.encoder.counts_per_cpr

    @property
    def encoder_position_cpr(self):
        """Return the position in counts per revolution."""
        return self._axis.encoder.pos_cpr

    @property
    def encoder_mode(self):
        """Return the mode of the connected encoder."""
        return self._axis.encoder.config.mode

    @encoder_mode.setter
    def encoder_mode(self, encoder_mode):
        """Set the encoder mode, see enums for possibilities.

        :param encoder_mode: The new encoder mode as integer.
        """
        if not isinstance(encoder_mode, int) or encoder_mode not in (0, 1):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(encoder_mode), nm='encoder_mode', rt='int (0, 1)'))
        self._axis.encoder.config.mode = encoder_mode
    # endregion

    # region Sensorless variables
    @property
    def sensorless_estimator_pm_flux_linkage(self):
        """Return the pm flux linkage, see odrive manual for the calculation."""
        return self._axis.sensorless_estimatro.config.pm_flux_linkage

    @sensorless_estimator_pm_flux_linkage.setter
    def sensorless_estimator_pm_flux_linkage(self, pm_flux_linkage):
        """Set a new pm flux linkage, see odrive manual for the calculation.

        :param pm_flux_linkage: The new pm flux linkage as float
        """
        if not isinstance(pm_flux_linkage, float):
            raise TypeError(self.TYPE_ERROR_MSG.format(gt=type(pm_flux_linkage), nm='pm_flux_linkage', rt='float'))
        self._axis.sensorless_estimatro.config.pm_flux_linkage = pm_flux_linkage
    # endregion
