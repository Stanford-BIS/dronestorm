MSP_RX_CONFIG = 44
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
MSP_SET_MOTOR = 214
MSP_EEPROM_WRITE = 250
MSP_DEBUG = 254

# payload byte lengths
# None means not implemented yet
MSP_PAYLOAD_LEN = {
    MSP_RX_CONFIG : 23,
    # MSP_IDENT : 7,
    MSP_STATUS : 15,
    MSP_RAW_IMU : 18,
    MSP_SERVO : None,
    MSP_MOTOR : 8,
    MSP_RC : {
        # RC data lengths vary with the receiver protocol used by the
        # flight control board
        "PWM" : 16,
        "PPM" : 24,
        "SPEKTRUM2048" : 24,
        "MSP" : 36,
    }, 
    MSP_RAW_GPS : 14,
    MSP_COMP_GPS : 5,
    MSP_ATTITUDE : 6,
    MSP_ALTITUDE : 6,
    MSP_ANALOG : 7,
    MSP_RC_TUNING : 7,
    MSP_PID : None,
    MSP_BOX : None,
    MSP_MISC : None,
    MSP_MOTOR_PINS : None,
    MSP_BOXNAMES : None,
    MSP_PIDNAMES : None,
    MSP_WP : 18,
    MSP_BOXIDS : None,
    MSP_RC_RAW_IMU : None,
    MSP_SET_RAW_RC : 12,
    MSP_SET_RAW_GPS : 14,
    MSP_SET_PID : None,
    MSP_SET_BOX : None,
    MSP_SET_RC_TUNING : 7,
    MSP_ACC_CALIBRATION : 0,
    MSP_MAG_CALIBRATION : 0,
    MSP_SET_MISC : None,
    MSP_RESET_CONF : 0,
    MSP_SET_WP : 18,
    MSP_SWITCH_RC_SERIAL : None,
    MSP_IS_SERIAL : None,
    MSP_SET_MOTOR : 8,
    MSP_EEPROM_WRITE : 0,
    MSP_DEBUG : None
}

# payload format strings
# None means not implemented yet
MSP_PAYLOAD_FMT = {
    MSP_RX_CONFIG : '<BHHHBHHBBHBIBB',
    # MSP_IDENT : '<BBBI',
    MSP_STATUS : '<HHHIBHH',
    MSP_RAW_IMU : '<9h',
    MSP_SERVO : None,
    MSP_MOTOR : '<HHHHHHHH',
    MSP_RC : '<18H',
    MSP_RC : {
        # RC data lengths vary with the receiver protocol used by the
        # flight control board
        "PWM" : '<'+str(MSP_PAYLOAD_LEN[MSP_RC]["PWM"]//2)+'H',
        "PPM" : '<'+str(MSP_PAYLOAD_LEN[MSP_RC]["PPM"]//2)+'H',
        "SPEKTRUM2048" : '<'+str(
            MSP_PAYLOAD_LEN[MSP_RC]["SPEKTRUM2048"]//2)+'H',
        "MSP" : '<'+str(MSP_PAYLOAD_LEN[MSP_RC]["MSP"]//2)+'H',
    }, 
    MSP_RAW_GPS : '<BBIIHHH',
    MSP_COMP_GPS : None,
    MSP_ATTITUDE : '<3h',
    MSP_ALTITUDE : '<ih',
    MSP_ANALOG : '<BHHH',
    MSP_RC_TUNING : '<BBBBBBB',
    MSP_PID : None,
    MSP_BOX : None,
    MSP_MISC : None,
    MSP_MOTOR_PINS : None,
    MSP_BOXNAMES : None,
    MSP_PIDNAMES : None,
    MSP_WP : '<BIIIHHB',
    MSP_BOXIDS : None,
    MSP_RC_RAW_IMU : None,
    MSP_SET_RAW_RC : '<18H',
    MSP_SET_RAW_GPS : '<BBIIHHH',
    MSP_SET_PID : None,
    MSP_SET_BOX : None,
    MSP_SET_RC_TUNING : '<BBBBBBB',
    MSP_ACC_CALIBRATION : '',
    MSP_MAG_CALIBRATION : '',
    MSP_SET_MISC : None,
    MSP_RESET_CONF : '',
    MSP_SET_WP : '<BIIIHHB',
    MSP_SWITCH_RC_SERIAL : None,
    MSP_IS_SERIAL : None,
    MSP_SET_MOTOR : '<HHHH',
    MSP_EEPROM_WRITE : '',
    MSP_DEBUG : None
}

