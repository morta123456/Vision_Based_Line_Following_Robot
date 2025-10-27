# import pypot.dynamixel

# ports = pypot.dynamixel.get_available_ports()

# if not ports:
#     raise IOError('no port found!')

# print('ports found', ports)

# with pypot.dynamixel.DxlIO(ports[0]) as dxl_io:
#     print('connected on the first available port:', ports[0])
#     print(dxl_io.scan(range(10)))
#     print(dxl_io.get_present_position((2, )))
#     dxl_io.set_goal_position({2: 50})
#     print(dxl_io.get_present_position((2, )))


import itertools
import numpy
import time

import pypot.dynamixel

AMP = 30
FREQ = 0.5

if __name__ == '__main__':
    ports = pypot.dynamixel.get_available_ports()
    print('available ports:', ports)

    if not ports:
        raise IOError('No port available.')

    port = ports[0]
    print('Using the first on the list', port)

    dxl_io = pypot.dynamixel.DxlIO(port)
    print('Connected!')

    found_ids = dxl_io.scan(range(10))
    print('Found ids:', found_ids)

    if len(found_ids) < 2:
        raise IOError('You should connect at least two motors on the bus for this test.')

    ids = found_ids[:2]

    dxl_io.enable_torque(ids)

    # speed = dict(zip(ids, itertools.repeat(200)))
    # dxl_io.set_moving_speed(speed)
    print(dxl_io.get_drive_mode(ids))
    pos = dict(zip(ids, itertools.repeat(70)))
    dxl_io.set_goal_position(pos)
    # speed = dict(zip(ids, itertools.repeat(0)))
    # dxl_io.set_moving_speed(speed)



    # dxl_io.set_goal_position(dict(zip(ids, itertools.repeat((50,50)))))
    # t0 = time.time()
    # while True:
    #     t = time.time()
    #     if (t - t0) > 5:
    #         break

    #     pos = AMP * numpy.sin(2 * numpy.pi * FREQ * t)
    #     dxl_io.set_goal_position(dict(zip(ids, itertools.repeat(pos))))

    #     time.sleep(0.02)
