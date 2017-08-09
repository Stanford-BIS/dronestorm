"""multiwii.py: Handle Multiwii Serial Protocol.

In the Multiwii serial protocol (MSP), packets are sent serially to and from
the flight control board.
Data is encoded in bytes using the little endian format.

Packets consist of
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
import serial, time, struct, subprocess, os

class MultiWii(object):
    """Multiwii Serial Protocol
    """
    MSP_IDENT = 100
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_SERVO = 103
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110
    MSP_RC_TUNING = 111
    MSP_PID = 112
    MSP_BOX = 113
    MSP_MISC = 114
    MSP_MOTOR_PINS = 115
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_WP = 118
    MSP_BOXIDS = 119
    MSP_RC_RAW_IMU = 121
    MSP_SET_RAW_RC = 200
    MSP_SET_RAW_GPS = 201
    MSP_SET_PID = 202
    MSP_SET_BOX = 203
    MSP_SET_RC_TUNING = 204
    MSP_ACC_CALIBRATION = 205
    MSP_MAG_CALIBRATION = 206
    MSP_SET_MISC = 207
    MSP_RESET_CONF = 208
    MSP_SET_WP = 209
    MSP_SWITCH_RC_SERIAL = 210
    MSP_IS_SERIAL = 211
    MSP_DEBUG = 254

    def __init__(self, serPort):
        self.PIDcoef = {
            'rp':0,'ri':0,'rd':0,
            'pp':0,'pi':0,'pd':0,
            'yp':0,'yi':0,'yd':0}
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0}
        self.rawIMU = {
            'ax':0,'ay':0,'az':0,
            'gx':0,'gy':0,'gz':0,
            'mx':0,'my':0,'mz':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0}
        self.attitude = {'angx':0,'angy':0,'heading':0}
        self.altitude = {'estalt':0,'vario':0}
        self.message = {
            'angx':0,'angy':0,'heading':0,
            'roll':0,'pitch':0,'yaw':0,'throttle':0}
        self.elapsed = 0

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
        try:
            self.ser.open()
        except(Exception) as error:
            print("\n\nError opening port "+self.ser.port +":")
            print(str(error)+"\n\n")
            raise(error)

    def compute_checksum(self, packet):
        """Computes the MSP checksum"""
        checksum = 0
        if type(packet[0]) is str: # python2 struct.pack returns string
            for i in packet[3:]:
                checksum ^= ord(i)
        else:                      # python3 struct.pack returns bytes of ints
            for i in packet[3:]:
                checksum ^= i
        return checksum

    def sendCMD(self, data_length, code, data):
        """Send a command to the board"""
        packet = (
            struct.pack('<ccc', b'$', b'M', b'<') +
            struct.pack('<BB%dH' % len(data), data_length, code, *data))
        packet += struct.pack('<B', self.compute_checksum(packet))
        try:
            self.ser.write(packet)
        except(Exception) as error:
            print("\n\nError sending command on port "+self.ser.port)
            print(str(error)+"\n\n")
            raise(error)

    def arm(self):
        """Arms the motors"""
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.sendCMD(8, MultiWii.MSP_SET_RAW_RC, data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    def disarm(self):
        """Disarms the motors"""
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.sendCMD(8,MultiWii.MSP_SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    def setPID(self,pd):
        data=[]
        for i in np.arange(1,len(pd),2):
            data.append(pd[i]+pd[i+1]*256)
        print("PID sending:", data)
        self.sendCMD(30, MultiWii.MSP_SET_PID, data)
        self.sendCMD(0, MultiWii.MSP_EEPROM_WRITE, [])

    def getData(self, cmd):
        """Function to receive a data packet from the board"""
        try:
            self.sendCMD(0, cmd, [])
            header = self.ser.read(3)
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength//2), data)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            if cmd == MultiWii.MSP_ATTITUDE:
                self.attitude['angx']=float(temp[0]/10.0)
                self.attitude['angy']=float(temp[1]/10.0)
                self.attitude['heading']=float(temp[2])
                return self.attitude
            elif cmd == MultiWii.MSP_ALTITUDE:
                self.altitude['estalt']=float(temp[0])
                self.altitude['vario']=float(temp[1])
                return self.rcChannels
            elif cmd == MultiWii.MSP_RC:
                self.rcChannels['roll']=temp[0]
                self.rcChannels['pitch']=temp[1]
                self.rcChannels['yaw']=temp[2]
                self.rcChannels['throttle']=temp[3]
                return self.rcChannels
            elif cmd == MultiWii.MSP_RAW_IMU:
                self.rawIMU['ax']=float(temp[0])
                self.rawIMU['ay']=float(temp[1])
                self.rawIMU['az']=float(temp[2])
                self.rawIMU['gx']=float(temp[3])
                self.rawIMU['gy']=float(temp[4])
                self.rawIMU['gz']=float(temp[5])
                self.rawIMU['mx']=float(temp[6])
                self.rawIMU['my']=float(temp[7])
                self.rawIMU['mz']=float(temp[8])
                return self.rawIMU
            elif cmd == MultiWii.MSP_MOTOR:
                self.motor['m1']=float(temp[0])
                self.motor['m2']=float(temp[1])
                self.motor['m3']=float(temp[2])
                self.motor['m4']=float(temp[3])
                return self.motor
            elif cmd == MultiWii.MSP_PID:
                dataPID=[]
                if len(temp)>1:
                    d=0
                    for t in temp:
                        dataPID.append(t%256)
                        dataPID.append(t//256)
                    for p in [0,3,6,9]:
                        dataPID[p]=dataPID[p]/10.0
                        dataPID[p+1]=dataPID[p+1]/1000.0
                    self.PIDcoef['rp']= dataPID=[0]
                    self.PIDcoef['ri']= dataPID=[1]
                    self.PIDcoef['rd']= dataPID=[2]
                    self.PIDcoef['pp']= dataPID=[3]
                    self.PIDcoef['pi']= dataPID=[4]
                    self.PIDcoef['pd']= dataPID=[5]
                    self.PIDcoef['yp']= dataPID=[6]
                    self.PIDcoef['yi']= dataPID=[7]
                    self.PIDcoef['yd']= dataPID=[8]
                return self.PIDcoef
            else:
                return "No return error!"
        except(Exception) as error:
            print("\n\nError in getData on port "+self.ser.port)
            print(str(error)+"\n\n")
            raise(error)

    def getDataInf(self, cmd):
        """Receive a data packet from the board.

        Note: easier to use on threads
        """
        while True:
            try:
                self.sendCMD(0,cmd,[])
                while True:
                    header = self.ser.read()
                    if header == '$':
                        header = header+self.ser.read(2)
                        break
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                temp = struct.unpack('<'+'h'*(datalength//2),data)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                if cmd == MultiWii.MSP_ATTITUDE:
                    self.attitude['angx']=float(temp[0]/10.0)
                    self.attitude['angy']=float(temp[1]/10.0)
                    self.attitude['heading']=float(temp[2])
                elif cmd == MultiWii.MSP_RC:
                    self.rcChannels['roll']=temp[0]
                    self.rcChannels['pitch']=temp[1]
                    self.rcChannels['yaw']=temp[2]
                    self.rcChannels['throttle']=temp[3]
                elif cmd == MultiWii.MSP_RAW_IMU:
                    self.rawIMU['ax']=float(temp[0])
                    self.rawIMU['ay']=float(temp[1])
                    self.rawIMU['az']=float(temp[2])
                    self.rawIMU['gx']=float(temp[3])
                    self.rawIMU['gy']=float(temp[4])
                    self.rawIMU['gz']=float(temp[5])
                elif cmd == MultiWii.MSP_MOTOR:
                    self.motor['m1']=float(temp[0])
                    self.motor['m2']=float(temp[1])
                    self.motor['m3']=float(temp[2])
                    self.motor['m4']=float(temp[3])
            except(Exception) as error:
                print("\n\nError in getDataInf on port "+self.ser.port)
                print(str(error)+"\n\n")
                raise(error)

    def closeSerial(self):
        """Close the serial port and reset the stty settings"""
        self.ser.close()
        bashCommand = "stty sane < /dev/ttyUSB0"
        os.system(bashCommand)
