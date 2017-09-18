"""Communicate with the Spektrum protocols over the Raspberry Pi 3 UART ports

Currently only implements the 2048 version of the Remote Receiver Protocol
used by Spektrum (AKA SPEKTRUM2048 in Betaflight).
Could be extended to other Spektrum protocols, specifically the Spektrum
Bidirectional Receiver Link (AKA SPEKTRUM Bidir SRXL in Betaflight).
"""
from __future__ import print_function
import time
import os
import struct
import serial
from .rc_util import (RC_MIN_SERIAL, RC_MID, RC_MAX_SERIAL)

N_CHAN = 12
MASK_CH_ID = 0b01111000 # 0x78
SHIFT_CH_ID = 3
MASK_SERVO_POS_HIGH = 0b00000111 # 0x07
SPEKTRUM_11MS_2048_DSMX_SYS_ID = 0xb2
CH_PER_PACKET = 7

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
        # consume the rest of the packet and we should be aligned with the
        # protocol now
        self.ser.read(15)

    def read_raw_channel_data(self):
        """Get the raw rc channel data from the receiver unit"""
        return self.ser.read(16)

    def write_raw_channel_data(self, data):
        """Write raw rc channel data"""
        return self.ser.write(data)

    @staticmethod
    def _serial_to_rc(serial_value):
        """Convert a serial value to an rc value

        Values transmitted over the remote receiver serial link range from
        0-2047 but rc values range from 988-2011
        Inputs
        ------
        serial_value : int
            value transmitted over the serial remote receiver protocol
        """
        rc_value = RC_MIN_SERIAL + (serial_value >> 1)
        return rc_value

    @staticmethod
    def _decode_serial_channel_data(ch_data):
        """Decode a channel's 2 byte remote receiver representation

        Inputs
        ------
        ch_data : 2 byte string (currently only supporting Python 2)
            Bytes within the remote receiver packet representing a channel's
            data

        Outputs
        -------
        channel_id, channel_data
        """
        ch_id = (ord(ch_data[0]) & MASK_CH_ID) >> SHIFT_CH_ID
        serial_val = (
            ((ord(ch_data[0]) & MASK_SERVO_POS_HIGH) << 8) | ord(ch_data[1]))
        rc_val = SpektrumRemoteReceiver._serial_to_rc(serial_val)
        return ch_id, rc_val

    def read_data(self):
        """Get the rc channel data from the receiver unit"""
        data_buf = self.read_raw_channel_data()
        data = data_buf[2:] # discard the header
        for i in range(7):
            ch_id, servo_pos = self._decode_serial_channel_data(
                data[2*i:2*i+2])
            # spektrum sends a mysterious, undocumented channel 12
            if ch_id < N_CHAN:
                self.rc_data[ch_id] = servo_pos
        return self.rc_data

    @staticmethod
    def _rc_to_serial(rc_val):
        """Convert an rc value to a serial value

        Values transmitted over the remote receiver serial link range from
        0-2047 but rc values range from 988-2011
        Inputs
        ------
        rc_value : int
            value to transmit over the serial remote receiver protocol
        """
        assert (rc_val >= RC_MIN_SERIAL) & (rc_val <= RC_MAX_SERIAL), (
            "Valid rc values %d-%d"%(RC_MIN_SERIAL, RC_MAX_SERIAL))
        serial_value = (rc_val - RC_MIN_SERIAL) << 1
        return serial_value

    @staticmethod
    def _encode_serial_channel_data(ch_id, rc_val):
        """Encode a channel's 2 byte remote receiver representation

        Inputs
        ------
        ch_id : int
            channel number 0-11
        ch_val : int
            channel's rc value 988-2011

        Outputs
        -------
        channel_data : 2 byte string (currently only supports Python 2)
        """
        serial_val = SpektrumRemoteReceiver._rc_to_serial(rc_val)
        ch_data = struct.pack('>H', (ch_id << 11) | serial_val)
        return ch_data

    def write_data(self, rc_data):
        """Send rc channel data

        Inputs
        ------
        rc_data : list-like
            rc data to send, indexed by channel
            valid values 988-2011
        """
        len_data = len(rc_data)
        assert len_data <= N_CHAN, (
            "Spektrum remote receiver protocol "+
            "supports only up to %d channels"%N_CHAN)

        # write header
        packet = struct.pack('>BB', 0, SPEKTRUM_11MS_2048_DSMX_SYS_ID)
        for ch_idx, rc_val in enumerate(rc_data):
            packet += self._encode_serial_channel_data(ch_idx, rc_val)
        if len_data < CH_PER_PACKET:
            for ch_idx in range(len_data, CH_PER_PACKET):
                packet += self._encode_serial_channel_data(ch_idx, RC_MID)
        self.write_raw_channel_data(packet)
