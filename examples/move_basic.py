import time
from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.155'

def main():
    arm = XArmAPI(ROBOT_IP)

    try:
        arm.clean_warn()
        arm.clean_error()
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(0)

        time.sleep(1)

        code = arm.set_position(
            x=250, y=0, z=150,
            roll=180, pitch=0, yaw=0,
            speed=50, wait=True
        )
        print('Naar startpose, code:', code)

        time.sleep(1)

        waypoints = [
            dict(x=250, y=  50, z=150),
            dict(x=250, y= -50, z=150),
            dict(x=250, y=   0, z=180),
        ]

        for i, p in enumerate(waypoints):
            code = arm.set_position(
                **p,
                roll=180, pitch=0, yaw=0,
                speed=50, wait=True
            )
            print(f'Waypoint {i}, code:', code)
            if code != 0:
                print('Beweging afgebroken door fout, code:', code)
                break

    finally:
        arm.set_state(4)  # pauze state
        arm.disconnect()
        print('Klaar, verbinding verbroken')

if __name__ == '__main__':
    main()
