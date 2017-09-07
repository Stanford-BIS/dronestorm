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
from . import msp_types as msp

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
    def __init__(self, port="/dev/ttyUSB0"):
        self.ser = serial.Serial()
        self.ser.port = port
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
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
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
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            raise error

    def get_rx_config(self):
        """Get the flight control board receiver configuration"""
        data = self.get_data(msp.MSP_RX_CONFIG)
        ret = {
            "serialrx_provider" : data[0],
            "maxcheck" : data[1],
            "midrc": data[2],
            "mincheck" : data[3],
            "spektrum_sat_bind" : data[4],
            "rx_min_usec" : data[5],
            "rx_max_usec" : data[6],
            "rcInterpolation" : data[7],
            "rcInterpolationInterval" : data[8],
            "airModeActivateThreshold" : data[9],
            "rx_spi_protocol" : data[10],
            "rx_spi_id" : data[11],
            "rx_spi_rf_channel_count" : data[12],
            "fpvCamAngleDegrees" : data[13],
        }
        return ret

    def get_status(self):
        """Get the flight control board status"""
        data = self.get_data(msp.MSP_STATUS)
        ret = {
            "cycle_time" : data[0],
            "i2c_error_count" : data[1],
            "sensor" : data[2],
            "flight_mode" : data[3],
            "profile" : data[4],
            "system_load" : data[5],
            "gyro_cycle_time" : data[6],
        }
        return ret

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
            bash_cmd = "stty sane < " + self.ser.port
            os.system(bash_cmd)
            raise error

    def close_serial(self):
        """Close the serial port and reset the stty settings"""
        self.ser.close()
        bash_cmd = "stty sane < " + self.ser.port
        os.system(bash_cmd)
