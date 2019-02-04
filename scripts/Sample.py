# -*- coding: utf-8 -*-


import sys
import time
import math
import os

#from pykeigan_motor import KMControllers
import KMControllers

def main():

    motor=None

    # ------------------------------#
    #   motor event
    # ------------------------------#

    # Notify torque, rotation, speed callback
    # on_motor_measurement_cb({'position':self.position,'velocity':self.velocity,'torque':self.torque})
    # To enable, specify interface (0b10001000) BLE: Disable USB: Enable
    def on_motor_measurement_cb(measurement):
        print('mt_position {} '.format(measurement))


    # Disconnect USB etc.
    def on_motor_connection_error_cb(e):
        global motor
        motor = None
        print(e)
        detect_usb_motor()

    # ------------------------------#
    #   function
    # ------------------------------#
    def get_kmSerialDev():
        kmSerialDev = False
        for path in os.listdir('/dev/'):
            if ('ttyUSB' in path):
                kmSerialDev = '/dev/' + path
        if not kmSerialDev:
            sys.stderr.write('Not motor attached\n')
        return kmSerialDev

    def setup_motor_usb_mode(motor):
        if (motor):
            motor.on_motor_measurement_cb = None
            print('Change usb mode >>>>>>>>')
            type = motor.interface_type['USB']+motor.interface_type['HDDBTN']
            motor.interface(type)  # 0b10001000
            motor.saveAllRegisters()
            time.sleep(1)
            motor.reboot()
            time.sleep(5)
            print('>>>>>>>>Comp Change usb mode')
        else:
            sys.stderr.write('Not motor attached\n')

    def reset_motor_usb_mode(motor):
        if (motor):
            motor.on_motor_measurement_cb = None
            print('Change usb mode >>>>>>>>')
            type = motor.interface_type['BLE']+motor.interface_type['I2C']+motor.interface_type['USB'] + motor.interface_type['HDDBTN']
            motor.interface(type)  # 0b10011001
            motor.saveAllRegisters()
            time.sleep(1)
            motor.reboot()
            time.sleep(5)
            print('>>>>>>>>Comp Change usb mode')
        else:
            sys.stderr.write('Not motor attached\n')


    def play_sequence(motor):
        if (motor):
            print('Play sequence >>>>>>>>')
            motor.on_motor_measurement_cb = on_motor_measurement_cb
            motor.stop()

            time.sleep(1)

            # Motor operation command. For details see https://en.document.keigan-motor.com/motor-control-command/command-motor-action.html
            motor.enable()
            #motor.presetPosition(0)
            motor.speed(1.0)
            motor.runForward()
            motor.wait(5000)  # Wait inside the motor
            motor.speed(1.0)
            motor.runReverse()
            motor.wait(5000)
            motor.stop()
            motor.free()

            time.sleep(12)
            print('>>>>>>>>Comp Play sequence')
            motor.on_motor_measurement_cb = None
        else:
            sys.stderr.write('Not motor attached\n')

    def check_motor_measurement(motor):
        print('Check motor measurement >>>>>>>>')
        print('Please turn the motor manually')
        motor.free()
        time.sleep(3)
        motor.on_motor_measurement_cb = on_motor_measurement_cb
        #Please turn the motor manually
        time.sleep(10)
        motor.on_motor_measurement_cb = None
        print('>>>>>>>>Comp motor measurement')

    def commandloop():
        global motor
        print("-----------------------------------------------------------------------")
        print("Setup motor usb mode:Setting required when acquiring the position information of the motor with USB. Once set it will be retained until reset.")
        print("Reset motor usb mode:Setting required when acquiring the position information of the motor other than USB.")
        print("Play sequence:Run the motor's motion.")
        print("Check motor measurement:Get the position information of the motor. Please rotate the motor by hand")
        print("-----------------------------------------------------------------------")
        print("Setup motor usb mode:[u] Reset motor usb mode:[r] Play sequence:[s] Check motor measurement:[m] exit:[x]")

        input_key = raw_input('>> ')
        #input_key = input('>> ')
        if (input_key in{"u","U"} ):
            setup_motor_usb_mode(motor)
        elif (input_key in{"r","R"}):
            reset_motor_usb_mode(motor)
        elif (input_key in{"s","S"}):
            play_sequence(motor)
        elif (input_key in{"m","M"}):
            check_motor_measurement(motor)
        # elif (input_key in {"d", "D"}):
        #     motor.delete()
        elif (input_key in{"x","X"}):
            exit()
        else:
            commandloop()

        commandloop()

    #MotorUSB connection
    def connection_usb_motor(kmSerialDev):
        global motor
        motor = KMControllers.USBController(kmSerialDev)
        print("connection_usb_motor")
        #motor.on_motor_measurement_cb = on_motor_measurement_cb
        motor.on_motor_connection_error_cb = on_motor_connection_error_cb
        commandloop()

    #LOOP until Motor is connected to USB
    def detect_usb_motor():
        while True:
            time.sleep(2)  # 2000ms
            kmSerialDev = get_kmSerialDev()
            if(kmSerialDev):
                connection_usb_motor(kmSerialDev)
                break

    detect_usb_motor()


if __name__ == '__main__':
    main()
