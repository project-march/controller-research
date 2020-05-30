

import rospy


TYPE_ERROR_MSG = 'Given type {gt} for {nm} is insufficient, requested type is {rt}'


class OdriveMotor(object):
    def __init__(self, odrive, odrive_axis):
        """Base class to communicate with a specific motor connected to a specified Odrive.

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
        return self._odrive.uptime

    @property
    def general_error(self):
        """Return the error state of the motor."""
        return self._axis.error

    @property
    def set_configuration(self):
        """Save the current configurations to the odrive."""
        self._odrive.save_configuration()

    @property
    def erase_configuration(self):
        """Delete the configuration on the odrive."""
        self._odrive.erase_configuration()

    @property
    def reboot(self):
        """Reboot the odrive."""
        self._odrive.reboot()
    # endregion

    # region Motor variables
    @property
    def motor_error(self):
        """Return the axis specific error."""
        return self._axis.motor.error

    @property
    def current_state(self):
        """Return the current state of the motor."""
        return self._axis.current_state

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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(watchdog_timeout), nm='watchdog_timeout', rt='float'))
        self._axis.config.watchdog_timeout = watchdog_timeout

    @property
    def pole_pairs(self):
        """Return the amount of pole pairs of the motor."""
        return self._axis.motor.config.pole_pairs

    @pole_pairs.setter
    def pole_pairs(self, amount_of_pole_pairs):
        """Set the amount of pole pairs of the attached motor.

        :param amount_of_pole_pairs: The amount of pole pairs as int32
        """
        if not isinstance(amount_of_pole_pairs, int):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(amount_of_pole_pairs), nm='amount_of_pole_pairs', rt='int32'))
        self._axis.config.pole_pairs = amount_of_pole_pairs

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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(phase_resistance), nm='phase_resistance', rt='float'))
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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(motor_type), nm='motor_type', rt='int (0, 1, 2)'))
        self._axis.motor.config.motor_type = motor_type
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
    def control_mode(self, control_mode):
        """Set the control mode of the motor using the control mode property.

        :param control_mode: The control mode as int8
        """
        if not isinstance(control_mode, int) or control_mode not in (0, 1, 2, 3, 4):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(control_mode), nm='control_mode', rt='int8 (0, 1, 2, 3, 4)'))
        self._axis.controller.config.control_mode = control_mode

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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(current_limit), nm='current_limit', rt='float'))
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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_pos_gain), nm='new_pos_gain', rt='float'))
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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_vel_gain), nm='new_vel_gain', rt='float'))
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
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_vel_limit), nm='new_vel_limit', rt='float'))
        self._axis.controller.config.vel_limit = new_vel_limit
    # endregion

    # region Actuating variables
    @property
    def position_setpoint(self):
        """Return the position setpoint of the controller."""
        return self._axis.controller.pos_setpoint

    @position_setpoint.setter
    def position_setpoint(self, new_pos_setpoint):
        """Set a new position setpoint to the axis.

        :param new_pos_setpoint: A new position setpoint as float
        """
        if not isinstance(new_pos_setpoint, float):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_pos_setpoint), nm='new_pos_setpoint', rt='float'))
        self._axis.controller.set_pos_setpoint(new_pos_setpoint)

    @property
    def velocity_setpoint(self):
        """Return the velocity setpoint of the controller."""
        return self._axis.controller.vel_setpoint

    @velocity_setpoint.setter
    def velocity_setpoint(self, new_vel_setpoint):
        """Set a new velocity setpoint to the axis.

        :param new_vel_setpoint: A new velocity setpoint as float
        """
        if not isinstance(new_vel_setpoint, float):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_vel_setpoint), nm='new_vel_setpoint', rt='float'))
        self._axis.controller.set_vel_setpoint(new_vel_setpoint)

    @property
    def current_setpoint(self):
        """Return the current setpoint of the controller."""
        return self._axis.controller.current_setpoint

    @current_setpoint.setter
    def current_setpoint(self, new_current_setpoint):
        """Set a new current setpoint to the axis.

        :param new_current_setpoint: A new current setpoint as float
        """
        if not isinstance(new_current_setpoint, float):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(new_current_setpoint), nm='new_current_setpoint', rt='float'))
        self._axis.controller.set_current_setpoint(new_current_setpoint)
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
        if not isinstance(encoder_mode, int) or encoder_mode not in (0, 1):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(encoder_mode), nm='encoder_mode', rt='int (0, 1)'))
        self._axis.encoder.config.mode = encoder_mode
    # endregion
