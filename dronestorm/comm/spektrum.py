"""Communicate with the Spektrum protocols over the Raspberry Pi 3 UART ports

Currently only implements the 2048 version of the Remote Receiver Protocol
used by Spektrum (AKA SPEKTRUM2048 in Betaflight).
Could be extended to other Spektrum protocols, specifically the Spektrum
Bidirectional Receiver Link (AKA SPEKTRUM Bidir SRXL in Betaflight).
"""
from __future__ import print_function
import time
import os
import serial

N_CHAN = 12
MASK_CH_ID = 0b01111000 # 0x78
SHIFT_CH_ID = 3
MASK_SERVO_POS_HIGH = 0b00000111 # 0x07

class SpektrumRemoteReceiver(object):
    """ Handle communication over Spektrum remote receiver protocol

    The remote receiver protocol is so named because it is the protocol used
    between Spektrum remote receivers (AKA external receivers and satellites)
    and Spektrum primary receivers (AKA internal receivers)

    Parameters
    ----------
    port : string (default "/dev/serial0")
        Location of port attached to the flight control board
        Will attempt to open upon initialization
    """
    def __init__(self, port='/dev/serial0'):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.open_serial()
        self.rc_data = [0] * N_CHAN

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

    def align_serial(self):
        """Aligns the serial stream with the incoming Spektrum packets

        Spektrum Remote Receivers (AKA Spektrum Satellite) communicate serially
        in 16 byte packets at 125000 bits per second (bps)(aka baud) but are
        compatible with the standard 115200bps rate. We don't control the
        output transmission timing of the Spektrum receiver unit and so might
        start reading from the serial port in the middle of a packet
        transmission. To align the reading from the serial port with the packet
        transmission, we use the timing between packets to detect the interval
        between packets.

        Packets are communicated every 11ms. At 115200 bps, a bit is read in
        approximately 8.69us, so a 16 byte (128 bit)
        packet will take around 1.11ms to be communicated, leaving a gap of
        about 9.89ms between packets. We align our serial port reading with
        the protocol by detecting this gap between reads.

        Note that we do not use the packet header contents because
            1) They are product dependent. Specifically, "internal" Spektrum
            receivers indicate the system protocol in the second byte of the
            header but "external" receivers do not. Further, different products
            are use different protocols and indicate this using the
            system protocol byte.
            2) Other bytes in the packet may take on the same value as the
            header contents. No bit patterns of a byte are reserved, so any
            byte in the data payload of the packet could match the values of
            the header bytes.

        Inputs
        ------
        ser: serial.Serial instance
            serial port to read from
        """
        # read in the first byte, might be a long delay in case the transmitter
        # is off when the program begins
        self.ser.read(1)
        dt_meas = 0
        # wait for the next long delay between reads
        dt_threshold = 0.005 # pick some threshold between 8.69us and 9.89ms
        while dt_meas < dt_threshold:
            start = time.time()
            self.ser.read()
            dt_meas = time.time()-start
        # consume the rest of the packet
        self.ser.read(15)
        # should be aligned with protocol now

    @staticmethod
    def _parse_channel_data(data):
        """Parse a channel's 2 bytes of data in a remote receiver packet

        Inputs
        ------
        data : 2 byte long string (currently only supporting Python 2)
            Bytes within the remote receiver packet representing a channel's
            data

        Outputs
        -------
        channel_id, channel_data
        """
        ch_id = (ord(data[0]) & MASK_CH_ID) >> SHIFT_CH_ID
        ch_data = (
            ((ord(data[0]) & MASK_SERVO_POS_HIGH) << 8) | ord(data[1]))
        ch_data = 988 + (ch_data >> 1)
        return ch_id, ch_data

    def get_data(self):
        """Get the rc channel data from the receiver unit"""
        data_buf = self.ser.read(16)
        data = data_buf[2:] # discard the header
        for i in range(7):
            ch_id, servo_pos = self._parse_channel_data(data[2*i:2*i+2])
            # spektrum sends a mysterious, undocumented channel 12
            if ch_id < N_CHAN:
                self.rc_data[ch_id] = servo_pos
        return self.rc_data

    def send_data(self, data):
        """Send rc channel data

        Inputs
        ------
        data : list-like data structure containing the current receiver data
            Use for performance reasons.
            If None, will generate a new list with each call
        """
        self.ser.write(data_buf)
