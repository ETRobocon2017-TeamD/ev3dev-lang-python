#!/usr/bin/env python3
# The MIT License (MIT)
#
# Copyright (c) 2016 Laurens Valk (laurensvalk@gmail.com)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
This is a class-based version of https://github.com/laurensvalk/segway
"""
import logging
import math
import time
from collections import deque
from ev3dev.auto import *
from ev3dev.helper import Tank


log = logging.getLogger(__name__)


########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)
    return int(infile.read().decode().strip())


# Function for fast writing to motor files
def FastWrite(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()


# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty, barance=1):
    # Clamp the value between -100 and 100
    duty = min(max(duty, -100), 100)*barance

    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, duty)


class GyroBalancer(Tank):
    """
    Base class for a robot that stands on two wheels and uses a gyro sensor
    to keep its balance.
    """

    def __init__(self,
                 gainGyroAngle,                   # For every radian (57 degrees) we lean forward,            apply this amount of duty cycle
                 gainGyroRate,                    # For every radian/s we fall forward,                       apply this amount of duty cycle
                 gainMotorAngle,                  # For every radian we are ahead of the reference,           apply this amount of duty cycle
                 gainMotorAngularSpeed,           # For every radian/s drive faster than the reference value, apply this amount of duty cycle
                 gainMotorAngleErrorAccumulated,  # For every radian x s of accumulated motor angle,          apply this amount of duty cycle
                 left_motor=OUTPUT_C,
                 right_motor=OUTPUT_B):
        Tank.__init__(self, left_motor, right_motor)

        # magic numbers
        self.gainGyroAngle                  = gainGyroAngle
        self.gainGyroRate                   = gainGyroRate
        self.gainMotorAngle                 = gainMotorAngle
        self.gainMotorAngularSpeed          = gainMotorAngularSpeed
        self.gainMotorAngleErrorAccumulated = gainMotorAngleErrorAccumulated

        # Sensor setup
        self.gyro = GyroSensor()
        self.gyro.mode = self.gyro.MODE_GYRO_RATE
        self.touch = TouchSensor()
        self.battery = PowerSupply()
        # self.remote = RemoteControl(channel=1)

        # if not self.remote.connected:
        #     log.error("%s is not connected" % self.remote)
        #     sys.exit(1)

        # Motor setup
        self.left_motor.reset()
        self.right_motor.reset()
        self.left_motor.run_direct()
        self.right_motor.run_direct()

        self.speed     = 0
        self.steering  = 0
        self.red_up    = False
        self.red_down  = False
        self.blue_up   = False
        self.blue_down = False
        self.STEER_SPEED = 20
        # self.remote.on_red_up    = self.make_move('red_up')
        # self.remote.on_red_down  = self.make_move('red_down')
        # self.remote.on_blue_up   = self.make_move('blue_up')
        # self.remote.on_blue_down = self.make_move('blue_down')

    def make_move(self, button):
        def move(state):
            # button pressed
            if state:
                if button == 'red_up':
                    self.red_up    = True
                elif button == 'red_down':
                    self.red_down  = True
                elif button == 'blue_up':
                    self.blue_up   = True
                elif button == 'blue_down':
                    self.blue_down = True

            # button released
            else:
                if button == 'red_up':
                    self.red_up    = False
                elif button == 'red_down':
                    self.red_down  = False
                elif button == 'blue_up':
                    self.blue_up   = False
                elif button == 'blue_down':
                    self.blue_down = False

            # forward
            if self.red_up and self.blue_up:
                self.speed = self.STEER_SPEED
                self.steering = 0

            # backward
            elif self.red_down and self.blue_down:
                self.speed = -1 * self.STEER_SPEED
                self.steering = 0

            # turn sharp right
            elif self.red_up and self.blue_down:
                self.speed = 0
                self.steering = -1 * self.STEER_SPEED * 2

            # turn right
            elif self.red_up:
                self.speed = 0
                self.steering = -1 * self.STEER_SPEED

            # turn sharp left
            elif self.red_down and self.blue_up:
                self.speed = 0
                self.steering = self.STEER_SPEED * 2

            # turn left
            elif self.blue_up:
                self.speed = 0
                self.steering = self.STEER_SPEED

            else:
                self.speed = 0
                self.steering = 0

            # log.info("button %8s, state %5s, speed %d, steering %d" % (button, state, self.speed, self.steering))

        return move

    def main(self):

        def shutdown():
            touchSensorValueRaw.close()
            gyroSensorValueRaw.close()
            batteryVoltageRaw.close()
            motorEncoderLeft.close()
            motorEncoderRight.close()
            motorDutyCycleLeft.close()
            motorDutyCycleRight.close()

            for motor in list_motors():
                motor.stop()

        try:

            ########################################################################
            ##
            ## Definitions and Initialization variables
            ##
            ########################################################################

            # Timing settings for the program
            loopTimeMilliSec        = 15                       # Time of each loop, measured in miliseconds.
            loopTimeSec             = loopTimeMilliSec/1000.0  # Time of each loop, measured in seconds.
            motorAngleHistoryLength = 3                        # Number of previous motor angles we keep track of.

            # Math constants
            radiansPerDegree               = math.pi/180       # The number of radians in a degree.

            # Platform specific constants and conversions
            degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
            radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
            degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
            radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
            RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled) EV3では、「speed」を1%とした場合、１分間に1.7回転させる速さであると定義する（※電圧によって変わる）
            degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed" 「speed」を回転角速度(deg)に変換する係数
            radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed" 「speed」を回転角速度(rad)に変換する係数

            # The rate at which we'll update the gyro offset (precise definition given in docs) ジャイロ値を補正するオフセット値の更新に使用する。0.00001745
            gyroDriftCompensationRate      = 0.1 * loopTimeSec * radiansPerSecondPerRawGyroUnit

            # A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
            motorAngleHistory = deque([0], motorAngleHistoryLength)

            # State feedback control gains (aka the magic numbers)
            gainGyroAngle                  = self.gainGyroAngle
            gainGyroRate                   = self.gainGyroRate
            gainMotorAngle                 = self.gainMotorAngle
            gainMotorAngularSpeed          = self.gainMotorAngularSpeed
            gainMotorAngleErrorAccumulated = self.gainMotorAngleErrorAccumulated

            battery_gain = 0.001089  # PWM出力算出用バッテリ電圧補正係数
            battery_offset = 0.625  # PWM出力算出用バッテリ電圧補正オフセット

            a_d = 1.0 - 0.51 #0.39 #0.47  # ローパスフィルタ係数(左右車輪の平均回転角度用)
            a_r = 0.985 #0.98  # ローパスフィルタ係数(左右車輪の目標平均回転角度用)

            # Variables representing physical signals (more info on these in the docs)
            # The angle of "the motor", measured in raw units (degrees for the
            # EV3). We will take the average of both motor positions as "the motor"
            # angle, wich is essentially how far the middle of the robot has traveled.
            motorAngleRaw              = 0

            # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
            motorAngle                 = 0

            # 1ループ前に算出した左右車輪回転角度
            motorAngleLast             = 0

            # The reference angle of the motor. The robot will attempt to drive
            # forward or backward, such that its measured position equals this
            # reference (or close enough).
            motorAngleReference        = 0

            # The error: the deviation of the measured motor angle from the reference.
            # The robot attempts to make this zero, by driving toward the reference.
            motorAngleError            = 0

            # We add up all of the motor angle error in time. If this value gets out of
            # hand, we can use it to drive the robot back to the reference position a bit quicker.
            motorAngleErrorAccumulated = 0

            # The motor speed, estimated by how far the motor has turned in a given amount of time
            motorAngularSpeed          = 0

            # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
            motorAngularSpeedReference = 0

            # The error: the deviation of the motor speed from the reference speed.
            motorAngularSpeedError     = 0

            # The 'voltage' signal we send to the motor. We calulate a new value each
            # time, just right to keep the robot upright.
            motorDutyCycle             = 0

            # The raw value from the gyro sensor in rate mode.
            gyroRateRaw                = 0

            # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
            gyroRate                   = 0

            # The gyro doesn't measure the angle of the robot, but we can estimate
            # this angle by keeping track of the gyroRate value in time
            gyroEstimatedAngle         = 0

            # Over time, the gyro rate value can drift. This causes the sensor to think
            # it is moving even when it is perfectly still. We keep track of this offset.
            gyroOffset                 = 0

            # 現在のバッテリー電圧
            voltageRaw = 0

            # ログ記録用
            logs = ["" for _ in range(10000)]
            log_pointer = 0

            # filehandles for fast reads/writes
            # =================================
            touchSensorValueRaw = open(self.touch._path + "/value0", "rb")
            gyroSensorValueRaw  = open(self.gyro._path + "/value0", "rb")
            batteryVoltageRaw = open(self.battery._path + "/voltage_now", "rb")

            # Open motor files for (fast) reading
            motorEncoderLeft    = open(self.left_motor._path + "/position", "rb")
            motorEncoderRight   = open(self.right_motor._path + "/position", "rb")

            # Open motor files for (fast) writing
            motorDutyCycleLeft  = open(self.left_motor._path + "/duty_cycle_sp", "w")
            motorDutyCycleRight = open(self.right_motor._path + "/duty_cycle_sp", "w")

            ########################################################################
            ##
            ## Calibrate Gyro
            ##
            ########################################################################
            print("-----------------------------------")
            print("Calibrating...")

            #As you hold the robot still, determine the average sensor value of 100 samples
            gyroRateCalibrateCount = 100
            for i in range(gyroRateCalibrateCount):
                gyroOffset = gyroOffset + FastRead(gyroSensorValueRaw)
                time.sleep(0.01)
            gyroOffset = gyroOffset/gyroRateCalibrateCount

            # Print the result
            print("GyroOffset: %s" % gyroOffset)
            print("-----------------------------------")
            print("GO!")
            print("-----------------------------------")

            #self.speed = 62.5 # 前進・後退速度。62.5〜-62.5の間で入力
            #self.steering = -1 * self.STEER_SPEED # 旋回速度。今の値が安定しているが、もう少し大きくしてもいいかも？

            ########################################################################
            ##
            ## MAIN LOOP (Press Touch Sensor to stop the program)
            ##
            ########################################################################

            # Initial touch sensor value
            touchSensorPressed = FastRead(touchSensorValueRaw)

            while not touchSensorPressed:
            #while ((gyroRate < 5) and (gyroRate > -5)):
            #for _ in range(1000):

                ###############################################################
                ##  Loop info
                ###############################################################
                tLoopStart = time.clock()

                ###############################################################
                ##  Reading the Remote Control
                ###############################################################
                # self.remote.process()

                ###############################################################
                ##  Reading the Gyro.
                ###############################################################
                gyroRateRaw = FastRead(gyroSensorValueRaw)
                gyroRate = (gyroRateRaw - gyroOffset) * radiansPerSecondPerRawGyroUnit # 躯体の角速度(rad/sec)。ジャイロから得た角速度をオフセット値で調整している

                ###############################################################
                ##  Reading the Motor Position
                ###############################################################
                motorAngleLast = motorAngle
                motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight))/2.0
                motorAngle = (motorAngleRaw * radiansPerRawMotorUnit) + gyroEstimatedAngle # 左右モーターの現在の平均回転角度(rad) + 躯体の(推定)回転角度

                motorAngularSpeedReference = self.speed * radPerSecPerPercentSpeed # 左右モーターの目標平均回転角速度(rad/sec)。入力値speedを角速度(rad)に変換したもの。
                motorAngleReference = motorAngleReference + (motorAngularSpeedReference * loopTimeSec) # 左右モーターの目標平均回転角度(rad)。初期値は0になる。入力値speedがずっと0でも0になる

                motorAngleError = motorAngle - motorAngleReference # 左右モーターの現在の平均回転角度と目標平均回転角度との誤差(rad)

                ###############################################################
                ##  Computing Motor Speed
                ###############################################################
                #motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(motorAngleHistoryLength * loopTimeSec) # 左右モーターの平均回転角速度(rad/sec)。３代前の左右モーターの平均回転角度と現在の平均回転角度の差分を、周期*3で割ったもの
                motorAngularSpeed = (((1.0 - a_d) * motorAngle) + (a_d * motorAngleLast) - motorAngleLast) / loopTimeSec # ローパスフィルター係数を利用
                motorAngularSpeedError = motorAngularSpeed - motorAngularSpeedReference # 左右モーターの現在の平均回転角速度と目標平均回転角速度との誤差(rad/sec)
                #motorAngleHistory.append(motorAngle) # 左右モーターの現在の平均回転角度を記録

                ###############################################################
                ##  Reading the Voltage.
                ###############################################################
                voltageRaw = FastRead(batteryVoltageRaw)

                ###############################################################
                ##  Computing the motor duty cycle value
                ###############################################################
                motorDutyCycle =(((gainGyroAngle  * gyroEstimatedAngle)
                               + (gainGyroRate   * gyroRate)
                               + (gainMotorAngle * motorAngleError)
                               + (gainMotorAngularSpeed * motorAngularSpeedError)
                               + (gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated))/
                                (battery_gain * voltageRaw - battery_offset)) * 100000

                # motorDutyCycle =(gainGyroAngle  * gyroEstimatedAngle
                #                + gainGyroRate   * gyroRate
                #                + gainMotorAngle * motorAngleError
                #                + gainMotorAngularSpeed * motorAngularSpeedError
                #                + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated)

                ###############################################################
                ##  Apply the signal to the motor, and add steering
                ###############################################################
                SetDuty(motorDutyCycleRight, motorDutyCycle + self.steering, 0.97)
                SetDuty(motorDutyCycleLeft, motorDutyCycle - self.steering)

                ###############################################################
                ##  Update angle estimate and Gyro Offset Estimate
                ###############################################################
                gyroEstimatedAngle = gyroEstimatedAngle + (gyroRate * loopTimeSec) # 次回の躯体の（推定）回転角度(rad)
                gyroOffset = ((1 - gyroDriftCompensationRate) * gyroOffset) + (gyroDriftCompensationRate * gyroRateRaw) # ジャイロの角速度を補正するオフセット値の更新。(rad/sec) ほぼほぼ前回算出したオフセット値寄りになる

                ###############################################################
                ##  Update Accumulated Motor Error
                ###############################################################
                motorAngleErrorAccumulated = motorAngleErrorAccumulated + (motorAngleError * loopTimeSec) # モーター角度誤差累積(rad*t) もしかして積分？ 前回の累積に、今回の目標平均回転角度との誤差を周期でかけたものを足している

                ###############################################################
                ##  Read the touch sensor (the kill switch)
                ###############################################################
                touchSensorPressed = FastRead(touchSensorValueRaw)

                # 実行時間、PWM値(duty cycle value)に関わる値をログに出力
                logs[log_pointer] = "%s, %s, %s, %s, %s, %s" % ((time.clock() - tLoopStart),
                    gainGyroAngle  * gyroEstimatedAngle,
                    gainGyroRate   * gyroRate,
                    gainMotorAngle * motorAngleError,
                    gainMotorAngularSpeed * motorAngularSpeedError,
                    gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated)
                log_pointer += 1

                ###############################################################
                ##  Busy wait for the loop to complete
                ###############################################################
                while ((time.clock() - tLoopStart) < loopTimeSec):
                    time.sleep(0.0001)

            shutdown()
            for log_ in logs:
                if log_ != "":
                    print(str(log_))

        # Exit cleanly so that all motors are stopped
        except (KeyboardInterrupt, Exception) as e:
            log.exception(e)
            shutdown()

    def set_moter_barance(self, duty_cycle_sp=100):

        def shutdown():
            touchSensorValueRaw.close()
            gyroSensorValueRaw.close()
            motorEncoderLeft.close()
            motorEncoderRight.close()
            motorDutyCycleLeft.close()
            motorDutyCycleRight.close()

            for motor in list_motors():
                motor.reset()

        try:

            ########################################################################
            ##
            ## Definitions and Initialization variables
            ##
            ########################################################################

            # Timing settings for the program
            loopTimeMilliSec        = 5000                       # Time of each loop, measured in miliseconds.
            loopTimeSec             = loopTimeMilliSec/1000.0  # Time of each loop, measured in seconds.
            motorAngleHistoryLength = 3                        # Number of previous motor angles we keep track of.

            # Math constants
            radiansPerDegree               = math.pi/180       # The number of radians in a degree.

            # Platform specific constants and conversions
            degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
            radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
            degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
            radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
            RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
            degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed"
            radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed"

            # The rate at which we'll update the gyro offset (precise definition given in docs)
            gyroDriftCompensationRate      = 0.1 * loopTimeSec * radiansPerSecondPerRawGyroUnit

            # A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
            motorAngleHistory = deque([0], motorAngleHistoryLength)

            # State feedback control gains (aka the magic numbers)
            gainGyroAngle                  = self.gainGyroAngle
            gainGyroRate                   = self.gainGyroRate
            gainMotorAngle                 = self.gainMotorAngle
            gainMotorAngularSpeed          = self.gainMotorAngularSpeed
            gainMotorAngleErrorAccumulated = self.gainMotorAngleErrorAccumulated

            # Variables representing physical signals (more info on these in the docs)
            # The angle of "the motor", measured in raw units (degrees for the
            # EV3). We will take the average of both motor positions as "the motor"
            # angle, wich is essentially how far the middle of the robot has traveled.
            motorAngleRaw              = 0

            # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
            motorAngle                 = 0

            # The reference angle of the motor. The robot will attempt to drive
            # forward or backward, such that its measured position equals this
            # reference (or close enough).
            motorAngleReference        = 0

            # The error: the deviation of the measured motor angle from the reference.
            # The robot attempts to make this zero, by driving toward the reference.
            motorAngleError            = 0

            # We add up all of the motor angle error in time. If this value gets out of
            # hand, we can use it to drive the robot back to the reference position a bit quicker.
            motorAngleErrorAccumulated = 0

            # The motor speed, estimated by how far the motor has turned in a given amount of time
            motorAngularSpeed          = 0

            # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
            motorAngularSpeedReference = 0

            # The error: the deviation of the motor speed from the reference speed.
            motorAngularSpeedError     = 0

            # The 'voltage' signal we send to the motor. We calulate a new value each
            # time, just right to keep the robot upright.
            motorDutyCycle             = 0

            # The raw value from the gyro sensor in rate mode.
            gyroRateRaw                = 0

            # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
            gyroRate                   = 0

            # The gyro doesn't measure the angle of the robot, but we can estimate
            # this angle by keeping track of the gyroRate value in time
            gyroEstimatedAngle         = 0

            # Over time, the gyro rate value can drift. This causes the sensor to think
            # it is moving even when it is perfectly still. We keep track of this offset.
            gyroOffset                 = 0

            motorAngleL = 0 # siori
            motorAngleR = 0

            # filehandles for fast reads/writes
            # =================================
            touchSensorValueRaw = open(self.touch._path + "/value0", "rb")
            gyroSensorValueRaw  = open(self.gyro._path + "/value0", "rb")

            # Open motor files for (fast) reading
            motorEncoderLeft    = open(self.left_motor._path + "/position", "rb")
            motorEncoderRight   = open(self.right_motor._path + "/position", "rb")

            # Open motor files for (fast) writing
            motorDutyCycleLeft  = open(self.left_motor._path + "/duty_cycle_sp", "w")
            motorDutyCycleRight = open(self.right_motor._path + "/duty_cycle_sp", "w")

            ########################################################################
            ##
            ## Calibrate Gyro
            ##
            ########################################################################
            print("-----------------------------------")
            print("Calibrating...")

            #As you hold the robot still, determine the average sensor value of 100 samples
            gyroRateCalibrateCount = 100
            for i in range(gyroRateCalibrateCount):
                gyroOffset = gyroOffset + FastRead(gyroSensorValueRaw)
                time.sleep(0.01)
            gyroOffset = gyroOffset/gyroRateCalibrateCount

            # Print the result
            print("GyroOffset: %s" % gyroOffset)
            print("duty_cycle_sp: %s" % duty_cycle_sp)
            power = PowerSupply()
            print("power: %s" % power.measured_volts)
            print("-----------------------------------")
            print("GO!")
            print("-----------------------------------")

            ########################################################################
            ##
            ## MAIN LOOP (Press Touch Sensor to stop the program)
            ##
            ########################################################################

            # Initial touch sensor value
            touchSensorPressed = FastRead(touchSensorValueRaw)
            mainTimeStart = time.clock()

            motorAngleLSpeedHistory = []
            motorAngleRSpeedHistory = []

            # while not touchSensorPressed:
            while not ((time.clock() - mainTimeStart) >  loopTimeSec):

                ###############################################################
                ##  Loop info
                ###############################################################
                tLoopStart = time.clock()

                ###############################################################
                ##  Reading the Gyro.
                ###############################################################
                gyroRateRaw = FastRead(gyroSensorValueRaw)
                gyroRate = (gyroRateRaw - gyroOffset)*radiansPerSecondPerRawGyroUnit

                ###############################################################
                ##  Reading the Motor Position
                ###############################################################
                motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight))/2
                motorAngle = motorAngleRaw*radiansPerRawMotorUnit

                motorAngularSpeedReference = self.speed * radPerSecPerPercentSpeed
                motorAngleReference = motorAngleReference + motorAngularSpeedReference * loopTimeSec

                motorAngleError = motorAngle - motorAngleReference

                ###############################################################
                ##  Computing Motor Speed
                ###############################################################
                motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(motorAngleHistoryLength * loopTimeSec)
                motorAngularSpeedError = motorAngularSpeed - motorAngularSpeedReference
                motorAngleHistory.append(motorAngle)

                ###############################################################
                ##  Computing the motor duty cycle value
                ###############################################################
                motorDutyCycle =(gainGyroAngle  * gyroEstimatedAngle
                               + gainGyroRate   * gyroRate
                               + gainMotorAngle * motorAngleError
                               + gainMotorAngularSpeed * motorAngularSpeedError
                               + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated)

                ###############################################################
                ##  Apply the signal to the motor, and add steering
                ###############################################################
                SetDuty(motorDutyCycleRight, duty_cycle_sp*1) # siori1
                SetDuty(motorDutyCycleLeft, duty_cycle_sp*1)

                t = time.clock()
                t = time.clock()

                motorAngleLRaw = FastRead(motorEncoderLeft)
                motorAngleLNew = motorAngleLRaw*radiansPerRawMotorUnit
                motorAngleLSpeed = motorAngleLNew - motorAngleL
                motorAngleL = motorAngleLNew
                motorAngleLSpeedHistory.append(motorAngleLSpeed)

                motorAngleRRaw = FastRead(motorEncoderRight)
                motorAngleRNew = motorAngleRRaw*radiansPerRawMotorUnit
                motorAngleRSpeed = motorAngleRNew - motorAngleR
                motorAngleR = motorAngleRNew
                motorAngleRSpeedHistory.append(motorAngleRSpeed)

                print("{t},{motorAngleLSpeed},{motorAngleRSpeed}".format(**locals()))

                ###############################################################
                ##  Update angle estimate and Gyro Offset Estimate
                ###############################################################
                gyroEstimatedAngle = gyroEstimatedAngle + gyroRate * loopTimeSec
                gyroOffset = (1 - gyroDriftCompensationRate) * gyroOffset + gyroDriftCompensationRate * gyroRateRaw

                ###############################################################
                ##  Update Accumulated Motor Error
                ###############################################################
                motorAngleErrorAccumulated = motorAngleErrorAccumulated + motorAngleError * loopTimeSec

                ###############################################################
                ##  Read the touch sensor (the kill switch)
                ###############################################################
                touchSensorPressed = FastRead(touchSensorValueRaw)

                ###############################################################
                ##  Busy wait for the loop to complete
                ###############################################################
                while ((time.clock() - tLoopStart) <  loopTimeSec/1000):
                    time.sleep(0.0001)

            motorAngleLSpeedAve = sum(motorAngleLSpeedHistory)/len(motorAngleLSpeedHistory)
            motorAngleRSpeedAve = sum(motorAngleRSpeedHistory)/len(motorAngleRSpeedHistory)
            print("motorAngleLSpeedAve:{motorAngleLSpeedAve}".format(**locals()))
            print("motorAngleRSpeedAve:{motorAngleRSpeedAve}".format(**locals()))
            shutdown()

        # Exit cleanly so that all motors are stopped
        except (KeyboardInterrupt, Exception) as e:
            log.exception(e)
            shutdown()
