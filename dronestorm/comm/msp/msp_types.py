# This module defines MSP message types, lengths, and formats

# MSP message types
MSP_RX_CONFIG = 44
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_WP = 118
MSP_SET_RAW_RC = 200
MSP_SET_RAW_GPS = 201
MSP_SET_RC_TUNING = 204
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_RESET_CONF = 208
MSP_SET_WP = 209
MSP_SET_MOTOR = 214
MSP_EEPROM_WRITE = 250

# Payload properties can depend on configuration
# Serial RX provider
RX_SERIAL_SPEKTRUM1024 = 0
RX_SERIAL_SPEKTRUM2048 = 1
RX_SERIAL_SBUS = 2
RX_SERIAL_SUMD = 3
RX_SERIAL_SUMH = 4
RX_SERIAL_XBUS_MODE_B = 5
RX_SERIAL_XBUS_MODE_B_RJ01 = 6
RX_SERIAL_IBUS = 7
RX_SERIAL_JETIEXBUS = 8
RX_SERIAL_CRSF = 9
RX_SERIAL_SPEKTRUM_SRXL = 10
# Other RX providers
RX_PWM = 21
RX_PPM = 22
RX_MSP = 23

# MSP payload byte lengths
MSP_PAYLOAD_LEN = {
    MSP_RX_CONFIG : 23,
    MSP_STATUS : 15,
    MSP_RAW_IMU : 18,
    MSP_MOTOR : 8,
    MSP_RC : {
        # RC data lengths vary with the receiver protocol used by the
        # flight control board
        RX_PWM : 16,
        RX_PPM : 24,
        RX_SERIAL_SPEKTRUM2048 : 24,
        RX_MSP : 36,
    }, 
    MSP_RAW_GPS : 14,
    MSP_ATTITUDE : 6,
    MSP_ALTITUDE : 6,
    MSP_ANALOG : 7,
    MSP_RC_TUNING : 7,
    MSP_WP : 18,
    MSP_SET_RAW_RC : 12,
    MSP_SET_RAW_GPS : 14,
    MSP_SET_RC_TUNING : 7,
    MSP_ACC_CALIBRATION : 0,
    MSP_MAG_CALIBRATION : 0,
    MSP_RESET_CONF : 0,
    MSP_SET_WP : 18,
    MSP_SET_MOTOR : 8,
    MSP_EEPROM_WRITE : 0,
}

# payload format strings
# None means not implemented yet
MSP_PAYLOAD_FMT = {
    MSP_RX_CONFIG : '<BHHHBHHBBHBIBB',
    MSP_STATUS : '<HHHIBHH',
    MSP_RAW_IMU : '<9h',
    MSP_MOTOR : '<HHHHHHHH',
    MSP_RC : '<18H',
    MSP_RC : {
        # RC data lengths vary with the receiver protocol used by the
        # flight control board
        RX_PWM : '<'+str(MSP_PAYLOAD_LEN[MSP_RC][RX_PWM]//2)+'H',
        RX_PPM : '<'+str(MSP_PAYLOAD_LEN[MSP_RC][RX_PPM]//2)+'H',
        RX_SERIAL_SPEKTRUM2048 : '<'+str(
            MSP_PAYLOAD_LEN[MSP_RC][RX_SERIAL_SPEKTRUM2048]//2)+'H',
        RX_MSP : '<'+str(MSP_PAYLOAD_LEN[MSP_RC][RX_MSP]//2)+'H',
    }, 
    MSP_RAW_GPS : '<BBIIHHH',
    MSP_ATTITUDE : '<3h',
    MSP_ALTITUDE : '<ih',
    MSP_ANALOG : '<BHHH',
    MSP_RC_TUNING : '<BBBBBBB',
    MSP_WP : '<BIIIHHB',
    MSP_SET_RAW_RC : '<18H',
    MSP_SET_RAW_GPS : '<BBIIHHH',
    MSP_SET_RC_TUNING : '<BBBBBBB',
    MSP_ACC_CALIBRATION : '',
    MSP_MAG_CALIBRATION : '',
    MSP_RESET_CONF : '',
    MSP_SET_WP : '<BIIIHHB',
    MSP_SET_MOTOR : '<HHHH',
    MSP_EEPROM_WRITE : '',
}

