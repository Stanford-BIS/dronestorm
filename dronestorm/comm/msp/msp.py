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
from .msp_types import MSP_PAYLOAD_LEN, MSP_PAYLOAD_FMT

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

    Parameters
    ----------
    port : string (default "/dev/ttyUSB0")
        Location of port attached to the flight control board
    rx_type : one of RX_* from msp_types (default RX_MSP)
        Indicate the type of protocol used to send receiver commands
        Current flight control board firmware does not provide complete
        information on receiver setup (only specifies which of the serial
        protocols (not including MSP) is used), so the user must provide this
        information for now...
    firmware_type : one of FIRMWARE_* from msp_types (default FIRMWARE_BF)
        Indicate the type of firmware used by the flight control board
        Current flight control board firmware does not provide information on
        firmware itself so the user must provide this information for now...
    """
    def __init__(self, port="/dev/ttyUSB0",
                 rx_protocol=msp.RX_MSP, firmware=msp.FIRMWARE_BF):

        assert rx_protocol in msp.RX_OPTIONS, (
            "unsupported rx protocol indicated")
        self.rx_protocol = rx_protocol
        self.rx_protocol_ch_count = len(
            MSP_PAYLOAD_FMT[msp.MSP_RC][rx_protocol][1:])

        assert firmware in msp.FIRMWARE_OPTIONS, (
            "unsupported firmware indicated")
        self.firmware = firmware
        self.firmware_motor_count = len(
            MSP_PAYLOAD_FMT[msp.MSP_MOTOR][firmware][1:])

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

    def open_serial(self):
        """Open the serial port"""
        try:
            self.ser.open()
        except(Exception) as error:
            print("\n\nError opening port "+self.ser.port +":")
            print(str(error)+"\n\n")
            # reset the stty settings
            bash_cmd = "stty sane < " + self.ser.port
            os.system(bash_cmd)
            raise error

    def close_serial(self):
        """Close the serial port and reset the stty settings"""
        self.ser.close()
        bash_cmd = "stty sane < " + self.ser.port
        os.system(bash_cmd)

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
            self.close_serial()
            raise error

    def get_data(self, cmd, data_len, fmt):
        """Function to request and receive a data packet from the board

        Checks that the received datapacket matches the request type and
        expected payload length. Does not have means to check that the payload
        format is as expected.

        Inputs
        ------
        cmd : int
            command as defined by one of the MSP_* identifiers in the
            msp_types module
        data_len : int
            data length as defined by one of the entries in MSP_PAYLOAD_LEN
            in the msp_types module
        fmt : string
            payload packing format string as defined by one of the entries
            in MSP_PAYLOAD_FMT in the msp_types module

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
            packet_data_len = self.ser.read(1)
            msg_type = self.ser.read(1)
            # handle python 2 vs 3 differences
            if isinstance(packet_data_len, str):
                packet_data_len = ord(packet_data_len)
                msg_type = ord(msg_type)
            else:
                packet_data_len = packet_data_len[0]
                msg_type = msg_type[0]

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
            assert packet_data_len == data_len, (
                "Unexpected MSP payload length. " +
                "Expected %d but received %d"%(data_len, packet_data_len))
            buf = self.ser.read(data_len)
            data = struct.unpack(fmt, buf)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except(Exception) as error:
            print("\n\nError in get_data on port "+self.ser.port)
            print(str(error)+"\n\n")
            self.close_serial()
            raise error
        return data

    def send_command(self, cmd, data_len, fmt, data):
        """Function to send a command to the board

        Inputs
        ------
        cmd : int
            command as defined by the MSP_* identifiers
        data_len : int
            data length as defined by one of the entries in MSP_PAYLOAD_LEN
            in the msp_types module
        fmt : string
            payload packing format string as defined by one of the entries
            in MSP_PAYLOAD_FMT in the msp_types module
        data : list
            data to be sent with the command; empty list if no data
        """
        try:
            # send command
            self.send_msg(data_len, cmd, data, fmt)
            # get acknowledgement
            ack_packet = self.ser.read(6)
            header = ack_packet[:3].decode() # [$, M, {<, >}, type, crc]
            direction = header[2]
            packet_data_length = ack_packet[3]
            msg_type = ack_packet[4]
            # handle python 2 vs 3 string vs bytes differences
            if isinstance(packet_data_length, str):
                packet_data_length = ord(packet_data_length)
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
            assert packet_data_length == 0, (
                "Unexpected MSP payload length in acknowledge packet. " +
                "Expected %d but received %d"%(0, packet_data_length))
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        except(Exception) as error:
            print("\n\nError in send_command on port "+self.ser.port)
            print(str(error)+"\n\n")
            self.close_serial()
            raise error

def get_rx_config(mw):
    """Get the flight control board receiver configuration

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_RX_CONFIG
    data = mw.get_data(msg, MSP_PAYLOAD_LEN[msg], MSP_PAYLOAD_FMT[msg])
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

def arm(mw):
    """Arms the motors

    Inputs
    ------
    mw: an instance of MultiWii
    """
    timer = 0
    start = time.time()
    while timer < 0.5:
        data = [1500, 1500, 2000, 1000]
        msg = msp.MSP_SET_RAW_RC
        mw.send_msg(8, msg, data, MSP_PAYLOAD_FMT[msg])
        time.sleep(0.05)
        timer = timer + (time.time() - start)
        start = time.time()

def disarm(mw):
    """Disarms the motors

    Inputs
    ------
    mw: an instance of MultiWii
    """
    timer = 0
    start = time.time()
    while timer < 0.5:
        data = [1500, 1500, 1000, 1000]
        msg = msp.MSP_SET_RAW_RC
        mw.send_msg(8, msg, data, MSP_PAYLOAD_FMT[msg])
        time.sleep(0.05)
        timer = timer + (time.time() - start)
        start = time.time()

def get_status(mw):
    """Get the flight control board status

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_STATUS
    data = mw.get_data(msg, MSP_PAYLOAD_LEN[msg], MSP_PAYLOAD_FMT[msg])
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

def get_attitude(mw):
    """Get the attitude data

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_STATUS
    data = mw.get_data(msg, MSP_PAYLOAD_LEN[msg], MSP_PAYLOAD_FMT[msg])
    data = mw.get_data(msp.MSP_ATTITUDE)
    roll = float(data[0]/10.0)
    pitch = float(data[1]/10.0)
    yaw = float(data[2])
    return roll, pitch, yaw

def get_altitute(mw):
    """Get the altitude data

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_STATUS
    return mw.get_data(msg, MSP_PAYLOAD_LEN[msg], MSP_PAYLOAD_FMT[msg])

def get_rc(mw):
    """Get the rc data

    Inputs
    ------
    mw: an instance of MultiWii

    Output
    -------
    list of [roll, pitch, yaw, throttlw, aux1, aux2, aux3, aux4]
    """
    msg = msp.MSP_RC
    return mw.get_data(
        msg,
        MSP_PAYLOAD_LEN[msg][mw.rx_protocol],
        MSP_PAYLOAD_FMT[msg][mw.rx_protocol])

def get_raw_imu(mw):
    """Get the raw imu data

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_RAW_IMU
    return mw.get_data(msg, MSP_PAYLOAD_LEN[msg], MSP_PAYLOAD_FMT[msg])

def get_motor(mw):
    """Get the motor data

    Inputs
    ------
    mw: an instance of MultiWii
    """
    msg = msp.MSP_MOTOR
    return mw.get_data(
        msg,
        MSP_PAYLOAD_LEN[msg][mw.firmware],
        MSP_PAYLOAD_FMT[msg][mw.firmware])

def set_pid(self, pid_coeff):
    """Set the PID coefficients

    Inputs
    ------
    mw: an instance of MultiWii
    """
    raise NotImplementedError

def set_rc(mw, data):
    """set the rc data

    Inputs
    ------
    mw: an instance of MultiWii
    data: int or list of ints
        rc values to set
        if list, can have up to the number of motors supported by the firmware
            if fewer than the numbe of supported motors, the rest of the motors
            will be set to midpoint values
        if int, will set all motors to that value
    """
    msg = msp.MSP_SET_RAW_RC
    assert isinstance(data, (int, list)), (
        "set_rc expects data to be list of ints or int"
    )
    if isinstance(data, list):
        len_data = len(data)
        assert len_data <= mw.rx_protocol_ch_count, (
            "rx protocol only supports up to %d channels"%(
                mw.rx_protocol_ch_count)
        )
        if len_data < mw.rx_protocol_ch_count:
            rc_midpoint = 1500
            data = data + [
                rc_midpoint for i in range(mw.rx_protocol_ch_count-len_data)]
    elif isinstance(data, int):
        data = [data for i in range(mw.rx_protocol_ch_count)]
    mw.send_command(
        msg,
        MSP_PAYLOAD_LEN[msg][mw.rx_protocol],
        MSP_PAYLOAD_FMT[msg][mw.rx_protocol],
        data)

def set_motor(mw, data):
    """Set the motor outputs

    Inputs
    ------
    mw: an instance of MultiWii
    data: int or list of ints
        motor values to set
        if list, can have up to the number of motors supported by the firmware
            if fewer than the numbe of supported motors, the rest of the motors
            will be set to 0
        if int, will set all motors to that value
    """
    msg = msp.MSP_SET_MOTOR
    assert isinstance(data, (int, list)), (
        "set_motor expects data to be list of ints or int"
    )
    if isinstance(data, list):
        len_data = len(data)
        assert len_data <= mw.firmware_motor_count, (
            "firmware only supports up to %d motors"%mw.firmware_motor_count
        )
        if len_data < mw.firmware_motor_count:
            data = data + [0 for i in range(mw.firmware_motor_count-len_data)]
    elif isinstance(data, int):
        data = [data for i in range(mw.firmware_motor_count)]
    mw.send_command(
        msg,
        MSP_PAYLOAD_LEN[msg][mw.firmware],
        MSP_PAYLOAD_FMT[msg][mw.firmware],
        data)
