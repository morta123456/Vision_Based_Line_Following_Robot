import pypot.dynamixel
import itertools
import time

import motor_controller

robot = motor_controller.MotorController()

print(robot.get_speed(2))
robot.set_speed(2,50)
time.sleep(5)
robot.motor_stop(2)



# ports = pypot.dynamixel.get_available_ports()
# print('available ports:', ports)
# port = ports[0]
# dxl_io = pypot.dynamixel.DxlIO(port)

# found_ids = dxl_io.scan(range(10))
# ids = found_ids[:2]


# speed = dict(zip(ids, [00,00]))
# dxl_io.set_moving_speed(speed)