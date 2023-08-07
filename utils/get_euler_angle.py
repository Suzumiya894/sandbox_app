import serial
import threading
import time
import ctypes
from typing import Optional
import utils.limomsg as limomsg

port_list = list(serial.tools.list_ports.comports())

class LimoFrame:
    def __init__(self,
                 stamp: float = 0.0,
                 port_id: ctypes.c_uint16 = 0,
                 count: ctypes.c_uint8 = 0,
                 data: Optional[bytearray] = None,
                 ):
        self.port_id = port_id
        self.stamp = stamp
        self.count = count
        #self.data = data
        if data is None:
            self.data = [0]*8
            #self.data = bytearray()
        elif isinstance(data, bytearray):
            self.data = data
        else:
            try:
                self.data = bytearray(data)
                #self.data = bytearray(data)
            except TypeError as error:
                err = f"Couldn't create message from {data} ({type(data)})"
                raise TypeError(err) from error

class LimoSerial:
    def __init__(self, name=None, baudrate=None):
        limomsg._init()
        if name == None:
            name = "/dev/ttyTHS1"
        elif len(port_list) == 0:
            print('no serial')
            # else serial.
        if baudrate == None:
            baudrate = "460800"
        self.serial_port = serial.Serial(name, baudrate, timeout=0.1)
        self.motion_mode = 0
        self.left_angle_scale_=2.47
        self.right_angle_scale_ = 1.64
        self.use_mcnamu = False
        with serial.Serial(name, baudrate, timeout=0.1) as receiver:
            serial_receive = threading.Thread(target=self.EnableAsynsSerial)
            serial_receive.start()
        time.sleep(0.2)

    def EnableAsynsSerial(self):

        while True:
            self.LimoGetFrame()

    def LimoSerialRead(self):

        if self.serial_port.in_waiting:
            read_data = self.serial_port.read(limomsg.LimoMsg.FRAME_LENGTH*2)

            if read_data[read_data.index(limomsg.LimoMsg.FRAME_HEADER)+limomsg.LimoMsg.FRAME_LENGTH] == limomsg.LimoMsg.FRAME_HEADER:
                data = read_data[read_data.index(
                    limomsg.LimoMsg.FRAME_HEADER):read_data.index(limomsg.LimoMsg.FRAME_HEADER)+limomsg.LimoMsg.FRAME_LENGTH]

            else:
                data = []

        else:
            data = []
        return data
    
    def LimoGetFrame(self):
        read_frame = LimoFrame()
        data = self.LimoSerialRead()
        if not data:
            return []
        read_frame.port_id = (data[2] << 8) | data[3]
        data_num = 0
        checksum = 0
        for data_num in range(8):
            read_frame.data[data_num] = data[data_num+4]
            checksum += data[data_num+4]

        if data[13] == (checksum % 256):
            if(read_frame.port_id == limomsg.LimoMsg.MSG_IMU_EULER_ID):
                imu_yaw = ((read_frame.data[1] & 0xff) | (read_frame.data[0] << 8)) / 100.0
                limomsg.SetIMUYaw(imu_yaw)
                imu_pitch = ((read_frame.data[3] & 0xff) | (read_frame.data[2] << 8)) / 100.0
                limomsg.SetIMUPitch(imu_pitch)
                imu_roll = ((read_frame.data[5] & 0xff) | (read_frame.data[4] << 8)) / 100.0
                limomsg.SetIMURoll(imu_roll)
        else:
            print('Invalid frame! Check sum failed!')
            return

class LIMO:
    '''
    GetIMUYawData()
    GetIMUPichData()
    GetIMURollData()
    '''
    def __init__(self, *device_args, **device_kwargs):
        serial_device_init_fn = LimoSerial.__init__
        args_names = serial_device_init_fn.__code__.co_varnames[:
                                                                serial_device_init_fn
                                                                .__code__.
                                                                co_argcount]
        args_dict = dict(zip(args_names, device_args))
        args_dict.update(device_kwargs)

        self.device = LimoSerial(**args_dict)

    def GetIMUYawData(self):
        return limomsg.GetIMUYaw()

    def GetIMUPitchData(self):
        return limomsg.GetIMUPitch()

    def GetIMURollData(self):
        return limomsg.GetIMURoll()

if __name__ == "__main__":
    limo = LIMO()
    Pitch_angle = limo.GetIMUPitchData()
    print(f"Pitch angle is {Pitch_angle}")
    Yaw_angle = limo.GetIMUYawData()
    print(f"Yaw angle is {Yaw_angle}")
    Roll_angle = limo.GetIMURollData()
    print(f"Roll angle is {Roll_angle}")

        
    
