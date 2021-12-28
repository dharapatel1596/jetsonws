from Arm_Lib import Arm_Device
import time


if __name__ == '__main__':
    Arm = Arm_Device()
    print(Arm.Arm_serial_servo_read(1))
    print(Arm.Arm_serial_servo_read(2))
    print(Arm.Arm_serial_servo_read(3))
    print(Arm.Arm_serial_servo_read(4))
    print(Arm.Arm_serial_servo_read(5))
    print(Arm.Arm_serial_servo_read(6))
   # while(1):
   #     Arm.Arm_serial_servo_write(6,135,400)
   #     time.sleep(1)
   
   
   #Arm.Arm_serial_servo_write(6,60,400)
   #     time.sleep(1)
