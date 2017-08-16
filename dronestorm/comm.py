"""
Library to communicate with the Naze32 flight control board.
Receives telemetry data: angx, angy and heading
Transmits attitude commands: roll, pitch, yaw
"""

from __future__ import division
from __future__ import print_function
import time
import struct
import os
import serial
import Adafruit_PCA9685
import pigpio
from dronestorm import msp_types as msp

class MultiWii(object):
    """Handle Multiwii Serial Protocol

    In the Multiwii serial protocol (MSP), packets are sent serially to and from
    the flight control board.
    Data is encoded in bytes using the little endian format.

    Packets consist of:
        Header   (3 bytes)
        Size     (1 byte)
        Type     (1 byte)
        Payload  (size indicated by Size portion of packet)
        Checksum (1 byte)

    Header is composed of 3 characters (bytes):
    byte 0: '$'
    byte 1: 'M'
    byte 2: '<' for data going to the flight control board or
            '>' for data coming from the flight contorl board

    Size is the number of bytes in the Payload
    Size can range from 0-255
    0 indicates no payload

    Type indicates the type (i.e. meaning) of the message
        Types values and meanings are mapped with MultiWii class variables

    Payload is the packet data
    Number of bytes must match the value of Size
    Specific formatting is specific to each Type

    Checksum is the XOR of all of the bits in [Size, Type, Payload]
    """
    def __init__(self, serPort="/dev/ttyUSB0"):
        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = None
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        self.open_serial()

    @staticmethod
    def _compute_checksum(packet):
        """Computes the MSP checksum

        Input
        -----
        packet: MSP packet without checksum created using struct.pack
        """
        checksum = 0
        if isinstance(packet[0], str):
            for i in packet[3:]:
                checksum ^= ord(i)
        else:
            for i in packet[3:]:
                checksum ^= i
        return checksum

    def send_msg(self, data_length, msg_type, data, data_format):
        """Send a message to the flight control board

        Inputs
        ------
        data_length: int
            length, in bytes, of the payload data
        msg_type: int
            message type identifier
            specified as one of the class variables
        data: list of data items
            data list contents specific to the message type
        data_format: string
            formatting string to use by struct.pack to pack data into packet
            mapped in class variable MSP_PAYLOAD_FMT
        """
        packet = (
            struct.pack('<ccc', b'$', b'M', b'<') +
            struct.pack('<BB', data_length, msg_type) +
            struct.pack(data_format, *data))
        packet += struct.pack('<B', self._compute_checksum(packet))
        try:
            self.ser.write(packet)
        except(Exception) as error:
            print("\n\nError sending command on port " + self.ser.port)
            print(str(error) + "\n\n")
            raise error

    def arm(self):
        """Arms the motors"""
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500, 1500, 2000, 1000]
            self.send_msg(
                8, msp.MSP_SET_RAW_RC, data,
                msp.MSP_PAYLOAD_FMT[msp.MSP_SET_RAW_RC])
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    def disarm(self):
        """Disarms the motors"""
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500, 1500, 1000, 1000]
            self.send_msg(
                8, msp.MSP_SET_RAW_RC, data, msp.MSP_PAYLOAD_FMT[msp.MSP_SET_RAW_RC])
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    def set_pid(self, pid_coeff):
        """Set the PID coefficients"""
        raise NotImplementedError

    def get_data(self, cmd):
        """Function to request and receive a data packet from the board

        Inputs
        ------
        cmd : int
            command as defined by the MSP_* identifiers

        Outputs
        -------
        data : list
            data decoded from serial stream according to MSP protocol
        """
        try:
            # send request
            self.send_msg(0, cmd, [], '')
            # get response
            header = self.ser.read(3).decode() # [$, M, {<, >}]
            direction = header[2]
            data_length = self.ser.read(1)
            msg_type = self.ser.read(1)
            # handle python 2 vs 3 differences
            if isinstance(data_length, str):
                data_length = ord(data_length)
                msg_type = ord(msg_type)
            else:
                data_length = data_length[0]
                msg_type = msg_type[0]

            print(data_length)
            # check that message received matches expected message
            assert msg_type == cmd, (
                "Unexpected MSP message type detected. " +
                "Received %d Expected %d"%(msg_type, cmd))
            assert direction != '!', (
                "Invalid MSP message direction '!' detected. " +
                "Indicates invalid request.")
            assert direction == '>', (
                "Unexpected MSP message direction. " +
                "Expected '<' but received '>'")
            buf = self.ser.read(data_length)
            data = struct.unpack(msp.MSP_PAYLOAD_FMT[cmd], buf)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except(Exception) as error:
            print("\n\nError in get_data on port "+self.ser.port)
            print(str(error)+"\n\n")
            raise error
        return data

    def send_command(self, cmd, data):
        """Function to send a command to the board

        Inputs
        ------
        cmd : int
            command as defined by the MSP_* identifiers
        data : list
            data to be sent with the command; empty list if no data
        """
        try:
            # send command
            self.send_msg(
                msp.MSP_PAYLOAD_LEN[cmd], cmd, data, msp.MSP_PAYLOAD_FMT[cmd])
            # get acknowledgement
            ack_packet = self.ser.read(6)
            header = ack_packet[:3].decode() # [$, M, {<, >}, type, crc]
            direction = header[2]
            data_length = ack_packet[3]
            msg_type = ack_packet[4]
            # handle python 2 vs 3 string vs bytes differences
            if isinstance(data_length, str):
                data_length = ord(data_length)
                msg_type = ord(msg_type)
            # check that message received matches expected message
            assert msg_type == cmd, (
                "Unexpected MSP message type detected. " +
                "Received %d Expected %d"%(msg_type, cmd))
            assert direction != '!', (
                "Invalid MSP message direction '!' detected. " +
                "Indicates invalid request.")
            assert direction == '>', (
                "Unexpected MSP message direction. " +
                "Expected '<' but received '>'")
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        except(Exception) as error:
            print("\n\nError in send_command on port "+self.ser.port)
            print(str(error)+"\n\n")
            raise error

    def get_status(self):
        """Get the flight control board status"""
        data = self.get_data(msp.MSP_STATUS)
        cycle_time = data[0]
        i2c_error_count = data[1]
        sensor = data[2]
        flight_mode = data[3]
        profile = data[4]
        system_load = data[5]
        gyro_cycle_time = data[6]
        return (cycle_time, i2c_error_count, sensor, flight_mode, profile,
                system_load, gyro_cycle_time)

    def get_attitude(self):
        """Get the attitude data"""
        data = self.get_data(msp.MSP_ATTITUDE)
        roll = float(data[0]/10.0)
        pitch = float(data[1]/10.0)
        yaw = float(data[2])
        return roll, pitch, yaw

    def get_altitute(self):
        """Get the altitude data"""
        return self.get_data(msp.MSP_ALTITUDE)

    def get_rc(self):
        """Get the rc data

        Output
        -------
        list of [roll, pitch, yaw, throttlw, aux1, aux2, aux3, aux4]
        """
        return self.get_data(msp.MSP_RC)

    def get_raw_imu(self):
        """Get the raw imu data"""
        return self.get_data(msp.MSP_RAW_IMU)

    def get_motor(self):
        """Get the motor data"""
        return self.get_data(msp.MSP_MOTOR)

    def set_rc(self, data):
        """set the rc data"""
        self.send_command(msp.MSP_SET_RAW_RC, data)

    def set_motor(self, data):
        """Set the motor outputs"""
        self.send_command(msp.MSP_SET_MOTOR, data)

    def open_serial(self):
        """Open the serial port"""
        try:
            self.ser.open()
        except(Exception) as error:
            print("\n\nError opening port "+self.ser.port +":")
            print(str(error)+"\n\n")
            raise error

    def close_serial(self):
        """Close the serial port and reset the stty settings"""
        self.ser.close()
        bash_cmd = "stty sane < /dev/ttyUSB0"
        os.system(bash_cmd)

class DroneComm(object):
    """Handles communication to and from the drone.

    Receives data over the USB with Multiwii protocol
    Transmits data via an I2C interface to an Adafruit PWM generator

    Paramters
    ---------
    pwm_ctrl: bool
        enable pwm control (default True)
    period: float
        pwm period (default 22ms)
    k_period: float
        pwm period calibration factor
    roll_pwm_trim: int
        roll channel trim (us)
    pitch_pwm_trim: int
        pitch channel trim (us)
    yaw_pwm_trim: int
        yaw channel trim (us)
    port: string
        usb port attached to flight control board (default "/dev/ttyUSB0")
        if None, assumes no input connection from flight control board
    """
    # Range of Values (Pulse Width): 1.1ms -> 1.9ms
    MIN_WIDTH = 0.0011
    MID_WIDTH = 0.0015
    MAX_WIDTH = 0.0019

    # Max change in pulse width from mid posision: 0.4ms
    MAX_DELTA_PWIDTH = 0.0004

    # time precision of feather pwm signal
    # each pwm cycle is divided into TICKS units of time
    TICKS = 4096

    # Featherboard channel map
    ROLL_CHANNEL = 3
    PITCH_CHANNEL = 2
    YAW_CHANNEL = 1
    THR_CHANNEL = 0

    # Calibration factor to compensate for mismatch between
    # requested pwm period and pwm freq implemented by Adafruit PWM generator
    # Useage:
    #     requested_period = K_PWM * target_period
    # when requesting a PWM signal with period target_period
    DEFAULT_K_PERIOD = 0.023 / 0.022 # 23ms/22ms

    def __init__(
            self, pwm_ctrl=True,
            period=0.022, k_period=None,
            roll_trim=0, pitch_trim=0, yaw_trim=0,
            port="/dev/ttyUSB0"):
        self.period = period

        # store trims in units of seconds
        self.trim = {
            'roll':roll_trim * 1E-6,
            'pitch':pitch_trim * 1E-6,
            'yaw':yaw_trim * 1E-6
        }

        self.attitude = {'roll':0, 'ptich':0, 'yaw':0}
        self.imu = {
            'ax':0, 'ay':0, 'az':0,
            'droll':0, 'dpitch':0, 'dyaw':0,
            'mx':0, 'my':0, 'mz':0}

        if k_period is None:
            k_period = self.DEFAULT_K_PERIOD

        if pwm_ctrl:
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(1./(k_period*period))
            self.reset_channels()
        else:
            self.pwm = None

        if port is None:
            self.board = None
        else:
            self.board = MultiWii(port)

    def reset_channels(self):
        """Reset channels 0-6 on the feather board

        Applies trim to roll/pitch/yaw channels
        """
        for i in range(6):
            self.set_pwidth(i, self.MID_WIDTH)
        self.set_pwidth(
            self.ROLL_CHANNEL, self.MID_WIDTH + self.trim['roll'])
        self.set_pwidth(
            self.PITCH_CHANNEL, self.MID_WIDTH + self.trim['pitch'])
        self.set_pwidth(
            self.YAW_CHANNEL, self.MID_WIDTH + self.trim['yaw'])

    def set_pwidth(self, channel, width):
        """Set a positive Pulse Width on a channel

        Parameters
        ----------
        channel: int
            pwm channel to set positive pulse width
        width: float
            positive pulse width (seconds)
        """
        width = width / self.period * self.TICKS
        pulse = int(round(width))
        self.pwm.set_pwm(channel, 0, pulse)

    def set_roll_pwidth(self, width):
        """Apply trim and set the pwm Roll signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['roll']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.ROLL_CHANNEL, width)
        if not valid:
            print("WARNING: Requested roll pwidth out of range!")

    def set_pitch_pwidth(self, width):
        """Apply trim and set the pwm Pitch signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['pitch']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.PITCH_CHANNEL, width)
        if not valid:
            print("WARNING: Requested pitch pwidth out of range!")

    def set_yaw_pwidth(self, width):
        """Apply trim and set the pwm Yaw signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        width += self.trim['yaw']
        width, valid = self.validate_pwidth(width)
        self.set_pwidth(self.YAW_CHANNEL, width)
        if not valid:
            print("WARNING: Requested yaw pwidth out of range!")

    def set_thr_pwidth(self, width):
        """Apply trim and set the pwm Throttle signal's positive pulse width

        Parameters
        ----------
        width: float
            positive pulse width (seconds)
        """
        # apply trim offset
        valid = True
        if width > self.MAX_WIDTH:
            valid = False
            width = self.MAX_WIDTH

        self.set_pwidth(self.THR_CHANNEL, width)
        if not valid:
            print("WARNING: Requested thr pwidth out of range!")

    def validate_pwidth(self, width):
        """Validate the pwm signal's positive pulse width

        Checks that the width is within the accepted range
        If not, the MAX_WIDTH or MIN_WIDTH is returned
        """
        if width > self.MAX_WIDTH:
            return self.MAX_WIDTH, False
        elif width < self.MIN_WIDTH:
            return self.MIN_WIDTH, False
        else:
            return width, True

    def set_roll_rate(self, rate):
        """Set the Roll rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['roll'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.ROLL_CHANNEL, width)
        if not valid:
            print("WARNING: Requested roll rate out of range!")

    def set_pitch_rate(self, rate):
        """Set the Pitch rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['pitch'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.PITCH_CHANNEL, width)
        if not valid:
            print("WARNING: Requested pitch rate out of range!")

    def set_yaw_rate(self, rate):
        """Set the Yaw rate

        Parameters
        ----------
        width: float [-1, 1]
            rate (scaled by max rate)
        """
        rate, valid = self.validate_rate(rate)
        width = (
            self.MID_WIDTH + self.trim['yaw'] + rate*self.MAX_DELTA_PWIDTH)
        self.set_pwidth(self.YAW_CHANNEL, width)
        if not valid:
            print("WARNING: Requested yaw rate out of range!")

    @staticmethod
    def validate_rate(rate):
        """Validate the requested channel rate is valid

        Checks that the width is within [-1, 1]
        Clips to range limit
        """
        if rate > 1:
            return 1, False
        elif rate < -1:
            return -1, False
        else:
            return rate, True

    def update_attitude(self):
        """Update the attitude data from the flight controller"""
        data = self.board.get_attitude()
        self.attitude['roll'] = data[0]
        self.attitude['pitch'] = data[1]
        self.attitude['yaw'] = data[2]

    def update_imu(self):
        """Updates the IMU data from the flight controller"""
        data = self.board.get_raw_imu()
        self.imu['ax'] = data[0]
        self.imu['ay'] = data[1]
        self.imu['az'] = data[2]
        self.imu['droll'] = data[3]
        self.imu['dpitch'] = data[4]
        self.imu['dyaw'] = data[5]
        self.imu['mx'] = data[6]
        self.imu['my'] = data[7]
        self.imu['mz'] = data[8]

    def exit(self):
        """
        Used to gracefully exit and close the serial port
        """
        if self.pwm is not None:
            self.reset_channels()
        if self.board is not None:
            self.board.close_serial()

class Reader:
    """A class to read PWM pulses and calculate their frequency
    and duty cycle.  The frequency is how often the pulse
    happens per second.  The duty cycle is the percentage of
    pulse high time per cycle.
    """
    def __init__(self, rpi, gpio, weighting=0.0):
        """
        Instantiate with the Pi and gpio of the PWM signal
        to monitor.

        Optionally a weighting may be specified.  This is a number
        between 0 and 1 and indicates how much the old reading
        affects the new reading.  It defaults to 0 which means
        the old reading has no effect.  This may be used to
        smooth the data.
        """
        self.rpi = rpi
        self.gpio = gpio

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting # Weighting for new reading.
        self._old = weighting       # Weighting for old reading.

        self._high_tick = None
        self._period = None
        self._high = None

        rpi.set_mode(gpio, pigpio.INPUT)

        self._cb = rpi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        """Callback function to trigger when level changes"""
        if level == 1:
            if self._high_tick is not None:
                t_diff = pigpio.tickDiff(self._high_tick, tick)
                if self._period is not None:
                    self._period = self._old*self._period + self._new*t_diff
                else:
                    self._period = t_diff
            self._high_tick = tick
        elif level == 0:
            if self._high_tick is not None:
                t_diff = pigpio.tickDiff(self._high_tick, tick)
                if self._high is not None:
                    self._high = self._old*self._high + self._new*t_diff
                else:
                    self._high = t_diff

    def frequency(self):
        """
        Returns the PWM frequency.
        """
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    def pulse_width(self):
        """
        Returns the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high / 1000000
        else:
            return 0.0

    def duty_cycle(self):
        """
        Returns the PWM duty cycle percentage.
        """
        if self._high is not None:
            return 100.0 * self._high / self._period
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()
