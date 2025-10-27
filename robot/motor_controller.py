from math import *
import numpy as np
import pypot.dynamixel
import itertools

class MotorController():

    def __init__(self):
        ports = pypot.dynamixel.get_available_ports()
        print('available ports:', ports)
        
        if not ports:
            raise IOError('No port available.')

        port = ports[0]
        print('Using the first on the list', port)

        self.dxl_io = pypot.dynamixel.DxlIO(port)
        print('Connected!')

        found_ids = self.dxl_io.scan(range(1,6))
        print('Found ids:', found_ids)
        
        if len(found_ids) < 2:
            raise IOError('You should connect at least two motors on the bus for this test.')

        self.ids = found_ids[:2]

    def speed_to_radial(self, speed):
        return (speed * 360) / (5.2 * np.pi)

    def radial_to_speed(self, radial):
        return (5.2 * np.pi * radial) / 360

    def get_speed(self, id):
        return self.radial_to_speed(self.dxl_io.get_present_speed([id])[0])

    def set_speed(self, id, speed):
        self.dxl_io.set_wheel_mode([id])
        self.dxl_io.set_moving_speed({id : self.speed_to_radial(speed)})

    # def get_postion(self, id):
    #     return self.dxl_io.get_present_position([id])

    def set_position(self, id, angle):
        self.dxl_io.set_joint_mode([id])
        self.dxl_io.set_goal_position(dict(zip([id], itertools.repeat(angle))))

    def motor_stop(self, id):
        self.set_speed(id, 0)
        # self.set_position(id,self.get_postion(id))
    
    def stop_all(self):
        self.dxl_io.set_wheel_mode(self.ids)
        self.dxl_io.set_moving_speed(dict(zip(self.ids, itertools.repeat(0))))
