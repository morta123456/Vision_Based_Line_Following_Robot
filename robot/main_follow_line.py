import time
from robotcontroller import RobotController
import cv2
import signal
import argparse

stream = None
controller = None
ok = True

def handle(_, _2):
    global ok
    if controller is not None:
        controller.panic()
        ok = False


def capture(stream):
    ret, frame = stream.read()
    if not ret:
        print("Failed to read camera stream")
        return None
    return frame

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", help="Activate debug mode", action="store_true")
    parser.add_argument("--linear-speed", help="Set max linear speed (cm/s)")
    parser.add_argument("--angular-speed", help="Set max angular speed (deg/s)")

    args = parser.parse_args()
    previous_time = time.monotonic() # in seconds
    delta = 0 # in seconds
    stream = cv2.VideoCapture(0)
    if not stream.isOpened():
        print("Cannot open camera")
        exit(-1)
    
    stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    stream.set(cv2.CAP_PROP_FRAME_WIDTH, 320)

    try:
        lspeed = int(args.linear_speed)
        aspeed = int(args.angular_speed)
        print(lspeed)
        print(aspeed)
        controller = RobotController(debug=args.d, linear_velocity=lspeed, angular_velocity=aspeed)
    except:
        print("Linear or angular speed invalid, using default values")
        controller = RobotController(debug=args.d)
    signal.signal(signal.SIGINT, handle)
    ok = True
    # Main loop
    while ok:
        delta = time.monotonic() - previous_time
        frame = capture(stream)
        if frame is None:
            controller.panic()
            break
        done = controller.tick_line_follow(frame, delta)
        if done:
            print("Loop done !")
            break
        if cv2.waitKey(1) == ord('q'):
            break
    cv2.destroyAllWindows()
    stream.release()