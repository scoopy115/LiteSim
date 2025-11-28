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

        start_x = 250
        start_z = 200
        size = 200 

        arm.set_position(x=start_x, y=size/2, z=start_z, roll=180, pitch=0, yaw=0, speed=60, wait=True)
        print('Klaar voor vierkantje...')

        for loop in range(2):
            print(f'Ronde {loop + 1}')
            
            arm.set_position(y=-size/2, speed=100, wait=True)
            
            arm.set_position(z=start_z + size, speed=100, wait=True)
            
            arm.set_position(y=size/2, speed=100, wait=True)
            
            arm.set_position(z=start_z, speed=100, wait=True)

    finally:
        arm.set_state(4)
        arm.disconnect()
        print('Vierkant voltooid')

if __name__ == '__main__':
    main()