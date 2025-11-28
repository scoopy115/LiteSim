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

        print('Naar zwaai-positie...')
        arm.set_position(x=200, y=0, z=300, roll=180, pitch=0, yaw=0, speed=80, wait=True)

        print('Start zwaaien...')
        for _ in range(3):
            arm.set_position(y=100, speed=150, wait=True)
            arm.set_position(y=-100, speed=150, wait=True)
        
        arm.set_position(y=0, speed=100, wait=True)

    finally:
        arm.set_state(4)
        arm.disconnect()
        print('Klaar met zwaaien')

if __name__ == '__main__':
    main()