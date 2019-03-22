#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 10 16:50:24 2017

@author: takata@innovotion.co.jp
"""

import serial, struct
import threading
import time
import atexit

import signal

def float2bytes(float_value):
    float_value = float(float_value)
    return struct.pack("!f", float_value)


def bytes2float(byte_array):
    return struct.unpack('!f', byte_array)[0]


def uint8_t2bytes(uint8_value):
    uint8_value = int(uint8_value)
    if uint8_value > 256 - 1:
        uint8_value = 256 - 1
    return struct.pack("B", uint8_value)


def uint16_t2bytes(uint16_value):
    uint16_value = int(uint16_value)
    if uint16_value > 256 ** 2 - 1:
        uint16_value = 256 ** 2 - 1
    val1 = int(uint16_value / 256)
    val2 = uint16_value - val1 * 256
    return struct.pack("BB", val1, val2)


def bytes2uint16_t(ba):
    return struct.unpack("BB", ba)[0]


def bytes2uint8_t(ba):
    return struct.unpack("B", ba)[0]


def bytes2int16_t(ba):
    return struct.unpack(">h", ba)[0]


def uint32_t2bytes(uint32_value):
    uint32_value = int(uint32_value)
    if uint32_value > 256 ** 4 - 1:
        uint32_value = 256 ** 4 - 1
    val1 = int(uint32_value / 256 ** 3)
    val2 = int((uint32_value - val1 * 256 ** 3) / 256 ** 2)
    val3 = int((uint32_value - val1 * 256 ** 3 - val2 * 256 ** 2) / 256)
    val4 = uint32_value - val1 * 256 ** 3 - val2 * 256 ** 2 - val3 * 256
    return struct.pack("BBBB", val1, val2, val3, val4)


class Controller:
    def __init__(self):
        pass

    def run_command(self, val, characteristics):
        print(val, characteristics)

    # Settings
    def maxSpeed(self, max_speed, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the maximum speed of rotation to the 'max_speed' in rad/sec.
        """
        command = b'\x02'
        values = float2bytes(max_speed)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def minSpeed(self, min_speed, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the minimum speed of rotation to the 'min_speed' in rad/sec.
        """
        command = b'\x03'
        values = float2bytes(min_speed)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def curveType(self, curve_type, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the acceleration or deceleration curve to the 'curve_type'.
        typedef enum curveType =
        {
            CURVE_TYPE_NONE = 0, // Turn off Motion control
            CURVE_TYPE_TRAPEZOID = 1, // Turn on Motion control with trapezoidal curve
        }
        """

        command = b'\x05'
        values = uint8_t2bytes(curve_type)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def acc(self, _acc, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the acceleration of rotation to the positive 'acc' in rad/sec^2.
        """
        command = b'\x07'
        values = float2bytes(_acc)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def dec(self, _dec, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the deceleration of rotation to the positive 'dec' in rad/sec^2.
        """
        command = b'\x08'
        values = float2bytes(_dec)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def maxTorque(self, max_torque, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the maximum torque to the positive 'max_torque' in N.m.
        """
        command = b'\x0E'
        values = float2bytes(max_torque)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def qCurrentP(self, q_current_p, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the q-axis current PID controller's Proportional gain to the postiive 'q_current_p'.
        """
        command = b'\x18'
        values = float2bytes(q_current_p)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def qCurrentI(self, q_current_i, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the q-axis current PID controller's Integral gain to the positive 'q_current_i'.
        """
        command = b'\x19'
        values = float2bytes(q_current_i)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def qCurrentD(self, q_current_d, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the q-axis current PID controller's Differential gain to the postiive 'q_current_d'.
        """
        command = b'\x1A'
        values = float2bytes(q_current_d)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def speedP(self, speed_p, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the speed PID controller's Proportional gain to the positive 'speed_p'.
        """
        command = b'\x1B'
        values = float2bytes(speed_p)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def speedI(self, speed_i, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the speed PID controller's Integral gain to the positive 'speed_i'.
        """
        command = b'\x1C'
        values = float2bytes(speed_i)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def speedD(self, speed_d, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the speed PID controller's Deferential gain to the positive 'speed_d'.
        """
        command = b'\x1D'
        values = float2bytes(speed_d)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def positionP(self, position_p, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the position PID controller's Proportional gain to the positive 'position_p'.
        """
        command = b'\x1E'
        values = float2bytes(position_p)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def resetPID(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Reset all the PID parameters to the firmware default settings.
        """
        command = b'\x22'
        self.run_command(command + identifier + crc16, 'motor_settings')

    def ownColor(self, red, green, blue, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the own LED color.
        """
        command = b'\x3A'
        values = uint8_t2bytes(red) + uint8_t2bytes(green) + uint8_t2bytes(blue)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def readRegister(self, register, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        '''
        Read a specified setting (register).
        '''
        command = b'\x40'
        values = uint8_t2bytes(register)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def saveAllRegisters(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Save all settings (registers) in flash memory.
        """
        command = b'\x41'
        self.run_command(command + identifier + crc16, 'motor_settings')

    def resetRegister(self, register, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Reset a specified register's value to the firmware default setting.
        """
        command = b'\x4E'
        values = uint8_t2bytes(register)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def resetAllRegisters(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Reset all registers' values to the firmware default setting.
        """
        command = b'\x4F'
        self.run_command(command + identifier + crc16, 'motor_settings')

    # Motor Action
    def disable(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Disable motor action.
        """
        command = b'\x50'
        self.run_command(command + identifier + crc16, 'motor_control')

    def enable(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Enable motor action.
        """
        command = b'\x51'
        self.run_command(command + identifier + crc16, 'motor_control')

    def speed(self, speed, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the speed of rotation to the positive 'speed' in rad/sec.
        """
        command = b'\x58'
        values = float2bytes(speed)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def presetPosition(self, position, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Preset the current absolute position as the specified 'position' in rad. (Set it to zero when setting origin)
        """
        command = b'\x5A'
        values = float2bytes(position)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def runForward(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Rotate the motor forward (counter clock-wise) at the speed set by 0x58: speed.
        """
        command = b'\x60'
        self.run_command(command + identifier + crc16, 'motor_control')

    def runReverse(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Rotate the motor reverse (clock-wise) at the speed set by 0x58: speed.
        """
        command = b'\x61'
        self.run_command(command + identifier + crc16, 'motor_control')

    # fix::[2019/01/29]runコマンドを追加
    def run(self, velocity, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Rotate the motor reverse (clock-wise) at the speed set by 0x58: speed.
        """
        command = b'\x62'
        values = float2bytes(velocity)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def moveTo(self, position, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Move the motor to the specified absolute 'position' at the speed set by 0x58: speed.
        """
        command = b'\x66'
        values = float2bytes(position)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def moveBy(self, distance, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Move motor by the specified relative 'distance' from the current position at the speed set by 0x58: speed.
        """
        command = b'\x68'
        values = float2bytes(distance)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def free(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Stop the motor's excitation
        """
        command = b'\x6C'
        self.run_command(command + identifier + crc16, 'motor_control')

    def stop(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Decelerate the speed to zero and stop.
        """
        command = b'\x6D'
        self.run_command(command + identifier + crc16, 'motor_control')

    def holdTorque(self, torque, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Keep and output the specified torque.
        """
        command = b'\x72'
        values = float2bytes(torque)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def doTaskSet(self, index, repeating, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Do taskset at the specified 'index' 'repeating' times.
        """
        command = b'\x81'
        values = uint16_t2bytes(index) + uint32_t2bytes(repeating)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def preparePlaybackMotion(self, index, repeating, option, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Prepare to playback motion at the specified 'index' 'repeating' times.
        """
        command = b'\x86'
        values = uint16_t2bytes(index) + uint32_t2bytes(repeating) + uint8_t2bytes(option)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def startPlaybackMotionV2(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):  # info::ver 1.18以降
        """
        Start to playback motion in the condition of the last preparePlaybackMotion.
        """
        command = b'\x85'
        self.run_command(command + identifier + crc16, 'motor_control')

    def startPlaybackMotion(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Start to playback motion in the condition of the last preparePlaybackMotion.
        """
        command = b'\x87'
        self.run_command(command + identifier + crc16, 'motor_control')

    def stopPlaybackMotion(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Stop to playback motion.
        """
        command = b'\x88'
        self.run_command(command + identifier + crc16, 'motor_control')

    # Queue
    def pause(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Pause the queue until 0x91: resume is executed.
        """
        command = b'\x90'
        self.run_command(command + identifier + crc16, 'motor_control')

    def resume(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Resume the queue.
        """
        command = b'\x91'
        self.run_command(command + identifier + crc16, 'motor_control')

    def wait(self, time, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Wait the queue or pause the queue for the specified 'time' in msec and resume it automatically.
        """
        command = b'\x92'
        values = uint32_t2bytes(time)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def reset(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Reset the queue. Erase all tasks in the queue. This command works when 0x90: pause or 0x92: wait are executed.
        """
        command = b'\x95'
        self.run_command(command + identifier + crc16, 'motor_control')

    # Taskset
    def startRecordingTaskset(self, index, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Start recording taskset at the specified 'index' in the flash memory.
        In the case of KM-1, index value is from 0 to 49 (50 in total).
        """
        command = b'\xA0'
        values = uint16_t2bytes(index)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def stopRecordingTaskset(self, index, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Stop recording taskset.
        This command works while 0xA0: startRecordingTaskset is executed.
        """
        command = b'\xA2'
        values = uint16_t2bytes(index)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def eraseTaskset(self, index, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Erase taskset at the specified index in the flash memory.
        In the case of KM-1, index value is from 0 to 49 (50 in total).
        """
        command = b'\xA3'
        values = uint16_t2bytes(index)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def eraseAllTaskset(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Erase all tasksets in the flash memory.
        """
        command = b'\xA4'
        self.run_command(command + identifier + crc16, 'motor_control')

    # Teaching
    def prepareTeachingMotion(self, index, time, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Prepare teaching motion by specifying the 'index' in the flash memory and recording 'time' in milliseconds.
        In the case of KM-1, index value is from 0 to 9 (10 in total).  Recording time cannot exceed 65408 [msec].
        """
        command = b'\xAA'
        values = uint16_t2bytes(index) + uint32_t2bytes(time)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def startTeachingMotionV2(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):  # info::ver 1.18以降
        """
        Start teaching motion in the condition of the last prepareTeachingMotion.
        This command works when the teaching index is specified by 0xAA: prepareTeachingMotion.
        """
        command = b'\xA9'
        self.run_command(command + identifier + crc16, 'motor_control')

    def startTeachingMotion(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Start teaching motion in the condition of the last prepareTeachingMotion.
        This command works when the teaching index is specified by 0xAA: prepareTeachingMotion.
        """
        command = b'\xAB'
        self.run_command(command + identifier + crc16, 'motor_control')

    def stopTeachingMotion(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Stop teaching motion.
        """
        command = b'\xAC'
        self.run_command(command + identifier + crc16, 'motor_control')

    def eraseMotion(self, index, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Erase motion at the specified index in the flash memory.
        In the case of KM-1, index value is from 0 to 9 (10 in total).
        """
        command = b'\xAD'
        values = uint16_t2bytes(index)
        self.run_command(command + identifier + values + crc16, 'motor_control')

    def eraseAllMotion(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Erase all motion in the flash memory.
        """
        command = b'\xAE'
        self.run_command(command + identifier + crc16, 'motor_control')

    # LED
    def led(self, ledState, red, green, blue, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Set the LED state (off, solid, flash and dim) and color intensity (red, green and blue).
        typedef enum ledState =
        {
            LED_STATE_OFF = 0, // LED off
            LED_STATE_ON_SOLID = 1, // LED solid
            LED_STATE_ON_FLASH = 2, // LED flash
            LED_STATE_ON_DIM = 3 // LED dim
        }
        """
        command = b'\xE0'
        values = uint8_t2bytes(ledState) + uint8_t2bytes(red) + uint8_t2bytes(green) + uint8_t2bytes(blue)
        self.run_command(command + identifier + values + crc16, "motor_control")

    # IMU
    def enableIMU(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Enable the IMU and start notification of the measurement values.
        This command is only available for BLE (not implemented on-wired.)
        When this command is executed, the IMU measurement data is notified to BLE IMU Measuement characteristics.
        """
        command = b'\xEA'
        self.run_command(command + identifier + crc16, 'motor_control')

    def disableIMU(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Disable the IMU and stop notification of the measurement values.
        """
        command = b'\xEB'
        self.run_command(command + identifier + crc16, 'motor_control')

    # System
    def reboot(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Reboot the system.
        """
        command = b'\xF0'
        self.run_command(command + identifier + crc16, 'motor_control')

    def enterDeviceFirmwareUpdate(self, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        """
        Enter the device firmware update mode.
        Enter the device firmware update mode or bootloader mode. It goes with reboot.
        """
        command = b'\xFD'
        self.run_command(command + identifier + crc16, 'motor_control')

class USBController(Controller):
    def __init__(self, port='/dev/ttyUSB0'):
        # signal.signal(signal.SIGINT, self.all_done)
        # signal.signal(signal.SIGTERM, self.all_done)
        self.serial_bef = []
        self.port = port
        self.serial = serial.Serial(port, 115200, 8, 'N', 1, None, False, True)
        self.on_motor_measurement_cb = False
        self.on_motor_connection_error_cb = False
        # info::BLEの通知をOFFにしてUSBで通知を受け取る。motor.saveAllRegisters()で保存しない場合はモーター再起動でデフォルトの接続経路(BLE > USB)に戻る
        self.interface(self.interface_type['USB'] + self.interface_type['HDDBTN'])
        self.t = threading.Thread(target=self._serial_schedule_worker)
        self.t.setDaemon(True)
        self.t.start()
        atexit.register(self.all_done)

    def all_done(self):
        try:
            if self.t.isAlive():
                self.t.join(0.01)
                self.serial.close()
        except:
            return

    def delete(self):
        try:
            if self.t.isAlive():
                self.t.join(0.01)
                self.serial.close()
        except:
            return

    def run_command(self, val, characteristics=None):
        try:
            self.serial.write(val)
        except serial.SerialException as e:
            self.serial.close()
            # There is no new data from serial port
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e
        except TypeError as e:
            # Disconnect of USB->UART occured
            self.serial.close()
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e
        except IOError as e:
            self.serial.close()
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e

    # ------------------------------#
    #   USBnotify切り替え モーター制御手段（インターフェイス）の設定(新 1-6 MOTOR_SETTING)
    # ------------------------------#

    # uint8_t flags ビットにより、含まれるパラメータを指定する（１の場合含む・0の場合含まない）
    # bit7	bit6	bit5	bit4	bit3	bit2	bit1	bit0
    # 物理			        有線  	有線			            無線
    # ボタン	＊	    ＊	    I2C	    USB	    ＊	    ＊	    BLE
    @property
    def interface_type(self):
        return {
            "BLE": 0b1,
            "USB": 0b1000,
            "I2C": 0b10000,
            "HDDBTN": 0b10000000,
        }

    def interface(self, flg=0b1000, identifier=b'\x00\x00', crc16=b'\x00\x00'):
        command = b'\x2E'
        values = uint8_t2bytes(flg)
        self.run_command(command + identifier + values + crc16, 'motor_settings')

    def _serial_schedule_worker(self):
        while True:
            time.sleep(100 / 1000)  # 100ms
            e_res = self._read_motor_measurement()
            if e_res:  # 例外発生でスレッド停止
                break

    def _read_motor_measurement(self):
        # rd = self.serial.read(self.serial.inWaiting())
        try:
            rd = self.serial.read(self.serial.inWaiting())
        except serial.SerialException as e:
            self.serial.close()
            # There is no new data from serial port
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e
        except TypeError as e:
            # Disconnect of USB->UART occured
            self.serial.close()
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e
        except IOError as e:
            self.serial.close()
            if (callable(self.on_motor_connection_error_cb)):
                self.on_motor_connection_error_cb(e)
            return e

        for bt in rd:
            if type(bt) is str:
                self.serial_bef.append(ord(bt))
            elif type(bt) is int:
                self.serial_bef.append(bt)

            # print bt.encode('hex')

        # ------------------------------#
        #   プリアンブル検出ロジック　#todo::バイト配列->バイト文字列で扱うように変更
        # ------------------------------#

        sv_len = len(self.serial_bef)
        is_pre = False  # プリアンブル検出したか
        if (sv_len < 8):
            return

        slice_idx = sv_len  # 抽出済みとしてバッファーから削除するインデックス
        bf_len = len(self.serial_bef)
        for i in range(bf_len):
            # プリアンブル検出
            if (i + 3 < bf_len and self.serial_bef[i] == 0x00 and self.serial_bef[i + 1] == 0x00 and self.serial_bef[
                i + 2] == 0xAA and self.serial_bef[i + 3] == 0xAA and not is_pre):
                is_pre = True
                slice_idx = i
                for ie in range(i + 4, sv_len, 1):
                    # ポストアンブル検出
                    if (ie + 3 < bf_len and self.serial_bef[ie + 2] == 0x0D and self.serial_bef[ie + 3] == 0x0A):
                        crc = self.serial_bef[ie] << 8 | self.serial_bef[ie + 1]  # CRC
                        payload = self.serial_bef[i + 4: ie]  # 情報バイト
                        val = self._serialdataParse(payload)
                        slice_idx = ie + 4
                        i = ie + 3
                        is_pre = False
                        # fix::[20190124harada]取得情報をモーター回転情報に制限
                        if (val['type'] == 0xB4 and callable(self.on_motor_measurement_cb)):
                            self.on_motor_measurement_cb(val['payload'])
                        break

        self.serial_bef = self.serial_bef[slice_idx:]

    def _serialdataParse(self, uint8List):
        v_len = len(uint8List)
        if (v_len < 3 or uint8List[0] != v_len):
            return {'type': None, 'payload': None}
        type = uint8List[1]
        payload = uint8List[2:]
        if type == 0xB4:  # モーター回転情報受信
            # todo::バイト配列->バイト文字列で扱うように変更
            pos_b = ''.join(map(lambda uint_num: struct.pack("B", uint_num), payload[0:4]))
            vel_b = ''.join(map(lambda uint_num: struct.pack("B", uint_num), payload[4:8]))
            tlq_b = ''.join(map(lambda uint_num: struct.pack("B", uint_num), payload[8:12]))
            self.position = bytes2float(pos_b)
            self.velocity = bytes2float(vel_b)
            self.torque = bytes2float(tlq_b)
            return {'type': type, 'payload': {'position': self.position, 'velocity': self.velocity,
                                              'torque': self.torque}}  # fix::[20190124harada]データタイプを追加
            # return {'type':type,'payload':{self.position,self.velocity,self.torque}}#fix::[20190124harada]データタイプを追加
        # if type == 0xB5:  #todo::ジャイロ情報
        # if type == 0xBE:  #todo::エラーコード情報
        # if type == 0x40:  #todo::レジスター読み取りコマンド実行時の返り値
        else:
            return {'type': None, 'payload': None}

