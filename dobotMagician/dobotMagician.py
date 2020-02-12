import struct
import threading
import datetime
import serial

from serial.tools import list_ports
from dobotMagician.message import Message
from _collections import deque

MODE_PTP_JUMP_XYZ = 0x00
MODE_PTP_MOVJ_XYZ = 0x01
MODE_PTP_MOVL_XYZ = 0x02
MODE_PTP_JUMP_ANGLE = 0x03
MODE_PTP_MOVJ_ANGLE = 0x04
MODE_PTP_MOVL_ANGLE = 0x05
MODE_PTP_MOVJ_INC = 0x06
MODE_PTP_MOVL_INC = 0x07
MODE_PTP_MOVJ_XYZ_INC = 0x08
MODE_PTP_JUMP_MOVL_XYZ = 0x09
MODE = MODE_PTP_MOVL_XYZ #Declare global Mode
TAG = 'DOBOT_MAGICIAN'

class DobotMagician:

    def __init__(self, port=None, con=False):
        self.con = con
        self.lock = threading.Lock()
        if port is None:
            ports = list_ports.comports()
            for i in ports:
                if i.vid in (4292, 6790):
                    print('DOBOT Magician is connected and ready to work', i, sep="\n")
                    port = i.device
                    break
            else:
                raise DobotException("Device not found!")

        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)

        if self.con:
            is_open = self.ser.isOpen()
            print(TAG, ': %s open' % self.ser.name if is_open else 'failed to open serial port')
        
        v = [100, 100, 100, 100]
        a = [50, 50, 50, 50]
        self._setQueuedCmdStartExec()
        self._setQueuedCmdClear()
        self._setPTPJointParams(v, a)
        self._setPTPCoordinateParams(velocity=100, acceleration=50)
        self._setPTPJumpParams(10, 200)
        self._setPTPCommonParams(velocity=100, acceleration=50)
        self._getPose()
        self._setModeL()
#-----------------------------------------------------------------------Programming / API methods------------------------------------------------------------
    def _setQueuedCmdStartExec(self):
        msg = Message()
        msg.id = 240
        msg.ctrl = 0x01
        return self._sendCMD(msg)
    
    def _setQueuedCmdClear(self):
        msg = Message()
        msg.id = 245
        msg.ctrl = 0x01
        return self._sendCMD(msg)

    def _sendCMD(self, msg):
        with self.lock:
            self.ser.reset_input_buffer()
            self._sendMSG(msg)
            return self._readMSG()

    def _sendMSG(self, msg):
        if self.con:
            print(TAG, ': -->', msg)
        self.ser.write(msg.bytes())

    def _readMSG(self):
        begin_found = False
        last_byte = None
        tries = 5
        while not begin_found and tries > 0:
            current_byte = ord(self.ser.read(1))
            if current_byte == 170:
                if last_byte == 170:
                    begin_found = True
            last_byte = current_byte
            tries = tries - 1
        if begin_found:
            payload_length = ord(self.ser.read(1))
            payload_checksum = self.ser.read(payload_length + 1)
            if len(payload_checksum) == payload_length + 1:
                b = bytearray([0xAA, 0xAA])
                b.extend(bytearray([payload_length]))
                b.extend(payload_checksum)
                msg = Message(b)
                if self.con:
                    print('Lenght', payload_length)
                    print(payload_checksum)
                    print('MessageID:', payload_checksum[0])
                    print(TAG, ': <--', ":".join('{:02x}'.format(x) for x in b))
                return msg
        return

    def _getPose(self):
        msg = Message()
        msg.id = 10
        response = self._sendCMD(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]
        if self.con:
            print(TAG, ': x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f' %
                    (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response
        
    def _getPoseL(self):
        msg = Message()
        msg.id = 13
        res = self._sendCMD(msg)
        poseL = struct.unpack_from('f', res.params, 0)[0]
        print(TAG, '-PoseL: ', poseL)
        x = self._get_device_L()
        return poseL

    def _setPTPJointParams(self, v, a):
        msg = Message()
        msg.id = 80
        msg.ctrl = 0x03
        msg.params = bytearray([])
        if(len(v) == 4 and len(a) == 4):
            for i in range(4):
                msg.params.extend(bytearray(struct.pack('f', v[i])))
            for i in range(4):
                msg.params.extend(bytearray(struct.pack('f', a[i])))
            return self._sendCMD(msg) 
        else:
            print(TAG, ': ERROR: Not enough parameters!')
        
    def _getPTPJointParams(self):
        msg = Message()
        msg.id = 80
        j = 0
        v, a = []
        for i in range(0, 12, 4):
            v[j] = struct.unpack_from('f', response.params, i)[0]
            j = j + 1
        j = 0
        for i in range(16, 28, 4):
            a[j] = struct.unpack_from('f', response.params, i)[0]
            j = j + 1
        return v, a
        
    def _setPTPJumpParams(self, jump, limit):
        msg = Message()
        msg.id = 82
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._sendCMD(msg)
    
    def _setPTPCoordinateParams(self, velocity, acceleration):
        msg = Message()
        msg.id = 81
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._sendCMD(msg)

    def _setPTPCommonParams(self, velocity, acceleration):
        msg = Message()
        msg.id = 83
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._sendCMD(msg)

    def _getQueuedCmdCurrentIndex(self):
        msg = Message()
        msg.id = 246
        response = self._sendCMD(msg)
        if response and response.id == 246:
            return self._extractCMDIndex(response)
        else:
            return -1

    def _extractCMDIndex(self, response):
        return struct.unpack_from('I', response.params, 0)[0]

    #Seriennummer des Geräts ausgeben lassen
    def _getDevSN(self):
        msg = Message()
        msg.id = 0
        return self._sendCMD(msg)
    
    def _waitForCmd(self, cmd_id):
        current_cmd_id = self._getQueuedCmdCurrentIndex()
        while cmd_id > current_cmd_id:
            if self.con:
                print("Current-ID", current_cmd_id)
                print("Waiting for", cmd_id)
            current_cmd_id = self._getQueuedCmdCurrentIndex()
    
    def _setModeL(self, enable=True):
        msg = Message()
        msg.id = 3
        msg.ctrl = 0x01
        msg.params = bytearray([])
        if(enable == True):
            msg.params.extend(bytearray([0x01]))
            msg.params.extend(bytearray([0x00]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._sendCMD(msg)

    def _setEndEffectorSuctionCup(self, enable=False):
        msg = Message()
        msg.id = 62
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._sendCMD(msg)

    def _setEndEffectorGripper(self, enable=False):
        msg = Message()
        msg.id = 63
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._sendCMD(msg)

    def _setEndEffectorLaser(self, power=255, enable=False):
        """Enables the laser. Power from 0 to 255. """
        msg = Message()
        msg.id = 61
        msg.ctrl = 0x03
        msg.params = bytearray([])
        # msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        # Assuming the last byte is power. Seems to have little effect
        msg.params.extend(bytearray([power]))
        return self._sendCMD(msg)

    def _setHomeCoordinate(self, x, y, z, r):
        msg = Message()
        msg.id = 30
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._sendCMD(msg)

    def _setHomeCmd(self):
        msg = Message()
        msg.id = 31
        msg.ctrl = 0x03
        msg.params = bytearray([])
        return self._sendCMD(msg)

    def _setArcCmd(self, x, y, z, r, cir_x, cir_y, cir_z, cir_r):
        msg = Message()
        msg.id = 101
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', cir_x)))
        msg.params.extend(bytearray(struct.pack('f', cir_y)))
        msg.params.extend(bytearray(struct.pack('f', cir_z)))
        msg.params.extend(bytearray(struct.pack('f', cir_r)))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._sendCMD(msg)

    def _moveTo(self, x, y, z, l, r, mode=MODE):
        self._setModeL()
        msg = Message()
        msg.id = 86
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        l=995 if(l>995) else 20 if(l<20) else l
        msg.params.extend(bytearray(struct.pack('f', l)))
        return self._sendCMD(msg)

    def _setPTPLParams(self, v, a):
        msg = Message()
        msg.id = 85
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v)))
        msg.params.extend(bytearray(struct.pack('f', a)))
        return self._sendCMD(msg)

    def _getPTPLParams(self):
        msg = Message()
        msg.id = 132
        response = self._sendCMD(msg)
        add = struct.unpack_from('I', response.params, 0)[0]
        v = struct.unpack_from('f', response.params, 4)[0]
        print(TAG, ' : add--> ', add)
        print(TAG, ' : freq--> ', v)
        #print(TAG, ' : duty--> ', a)
        return response

#  -----------------------------------------------------------------------------User methods------------------------------------------------------------------------------------------------
    def speed(self, velocity=50., acceleration=50.):
        self._waitForCmd(self._extractCMDIndex(self._setPTPCommonParams(velocity, acceleration)))
        self._waitForCmd(self._extractCMDIndex(self._setPTPCoordinateParams(velocity, acceleration)))

    def laser(self, power=0):
        return self._extractCMDIndex(self._setEndEffectorLaser(power, True))

    def stopLaser(self):
        return self._extractCMDIndex(self._setEndEffectorLaser(False))

    def grip(self):
        return self._extractCMDIndex(self._setEndEffectorGripper(True))

    def stopGrip(self):
        return self._extractCMDIndex(self._setEndEffectorGripper())

    def suck(self):
        return self._extractCMDIndex(self._setEndEffectorSuctionCup(True))

    def stopSuck(self):
        return self._extractCMDIndex(self._setEndEffectorSuctionCup())

    def goArc(self, x, y, z, r, cir_x, cir_y, cir_z, cir_r):
        return self._extractCMDIndex(self._setArcCmd(x, y, z, r, cir_x, cir_y, cir_z, cir_r))

    def setHome(self):
        return self._extractCMDIndex(self._setHomeCmd())

    def getPose(self):
        response = self._getPose()
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4

    def moveTo(self, x, y, z, l=None, r=0., mode=MODE):
        return self._waitForCmd(self._extractCMDIndex(self._moveTo(x, y, z, l, r)))

    def getPoseL(self):
        msg = Message()
        msg.id = 13
        res = self._sendCMD(msg)
        poseL = struct.unpack_from('f', res.params, 0)[0]
        print(TAG, ': <<<< poseL: ', poseL)
        return poseL
                              
# ---------------------------------------------------------------Destructor---------------------------------------------------------'''

    def __del__(self):
        with self.lock:
            self.ser.close()
            if self.con:
                print(TAG, ': %s closed' % self.ser.name)