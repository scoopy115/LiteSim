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

        print('Naar startpositie (joints)...')
        arm.set_servo_angle(angle=[0, 0, 90, 0, 90, 0], speed=40, wait=True)

        print('Start slang-dans...')
        for _ in range(3):
            arm.set_servo_angle(angle=[45, 0, 90, 0, 60, 0], speed=60, wait=True)
            
            arm.set_servo_angle(angle=[-45, 0, 90, 0, 120, 0], speed=60, wait=True)

        arm.set_servo_angle(angle=[0, 0, 90, 0, 90, 0], speed=40, wait=True)

    finally:
        arm.set_state(4)
        arm.disconnect()
        print('Dans voltooid')

if __name__ == '__main__':
    main()