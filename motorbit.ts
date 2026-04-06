/*
modified from pxt-servo/servodriver.ts
load dependency
"motorbit": "file:../pxt-motorbit"
*/
enum Offset {
    //% block=one
    ONE = 0,
    //% block=two
    TWO = 1,
    //% block=three
    THREE = 2,
    //% block=four
    FOUR = 3
}

//% color="#EE6A50" weight=10 icon="\uf0d1"
namespace motorbit {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    export enum Motors {
        M1 = 0x1,
        M2 = 0x2,
        M3 = 0x3,
        M4 = 0x4
    }

    export enum Steppers {
        STPM1_2 = 0x2,
        STPM3_4 = 0x1
    }

    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum TurnMode {
        //% block="2 motors (Tank)"
        Tank = 0,
        //% block="1 motor (Pivot)"
        Pivot = 1
    }

    export enum DistanceUnit {
        //% block="cm"
        CM = 0,
        //% block="inch"
        Inch = 1
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }

    let initialized = false
    let matBuf = pins.createBuffer(17);
    let distanceBuf = 0;

    let _dt_leftMotor: motorbit.Motors = motorbit.Motors.M3;
    let _dt_rightMotor: motorbit.Motors = motorbit.Motors.M4;
    let _dt_leftWheelDia: number = 4.8;
    let _dt_rightWheelDia: number = 4.8;
    let _dt_trackWidth: number = 8.8;
    let _dt_ticksPerRev: number = 270;

    let _dt_leftCount = 0;
    let _dt_rightCount = 0;

    let _start_heading = 0;
    let _dt_turnK = 1.00

    // Motor direction: 1 = normal, -1 = inverted (set via setMotorDirection block)
    let _dt_leftInvert: number = -1;
    let _dt_rightInvert: number = 1;   // right motor is physically reversed by default

    // IMU yaw direction: 1 = normal, -1 = inverted (set if BNO055 is mounted upside-down)
    let _dt_imuInvert: number = 1;

    // Tuning parameters — change with tune blocks in Robot Setup
    let _tune_driveKp: number = 1.5;      // encoder balance gain for driveDistance
    let _tune_driveHdgKp: number = 3.5;   // IMU heading correction gain for driveStraightTo
    let _tune_imuTurnKp: number = 2.2;    // IMU turn proportional gain
    let _tune_imuTurnKd: number = 8.0;    // IMU turn derivative gain (reduces overshoot)
    let _tune_encTurnKp: number = 0.6;    // encoder balance gain for tankTurn
    let _tune_trimSlowCmd: number = 32;   // max speed in slow zone before TRIM_ZONE
    let _tune_trimDecelMs: number = 150;  // ms to wait (motors off) before first micro-pulse
    let _tune_trimCmd: number = 70;       // motor command used for micro-pulses
    let _tune_trimPulseMin: number = 3;   // minimum micro-pulse duration (ms)
    let _tune_trimPulseMax: number = 5;   // maximum micro-pulse duration (ms)

    // ==========================================
    // IMU Sensor (BNO055)
    // ==========================================
    const BNO055_ADDR = 0x28;
    const BNO_REG_CHIP_ID = 0x00;
    const BNO_REG_OPR_MODE = 0x3D;
    const BNO_REG_UNIT_SEL = 0x3B;
    const BNO_REG_EUL_HEADING_LSB = 0x1A;
    const BNO_REG_CALIB_STAT = 0x35;

    const OP_MODE_CONFIG = 0x00;
    const OP_MODE_IMU = 0x08;
    const OP_MODE_NDOF = 0x0C;

    let _imu_heading_offset = 0;
    let _imu_initialized = false;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval;
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10;
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode);
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    // ==========================================
    // driveMotors — single place to fix right motor direction
    // leftSpeed / rightSpeed range: -255 to 255
    // ==========================================
    function driveMotors(leftSpeed: number, rightSpeed: number): void {
        MotorRun(_dt_leftMotor, leftSpeed * _dt_leftInvert)
        MotorRun(_dt_rightMotor, rightSpeed * _dt_rightInvert)
    }

    // Kick-start motors to overcome static friction (stiction).
    // If target speed is below threshold, briefly runs at KICK_CMD first,
    // then drops to the target speed. Helps motors start smoothly at low speeds.
    function kickMotors(leftSpeed: number, rightSpeed: number): void {
        const KICK_CMD = 50
        const KICK_MS = 10
        const THRESHOLD = 50  // only kick if target speed is below this
        let maxAbs = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed))
        if (maxAbs > 0 && maxAbs < THRESHOLD) {
            driveMotors(
                leftSpeed > 0 ? KICK_CMD : leftSpeed < 0 ? -KICK_CMD : 0,
                rightSpeed > 0 ? KICK_CMD : rightSpeed < 0 ? -KICK_CMD : 0
            )
            basic.pause(KICK_MS)
        }
        driveMotors(leftSpeed, rightSpeed)
    }

    // ==========================================
    // Gorilla Go — simplified blocks for kids
    // ==========================================

    /**
     * Set up the robot for MotorBit V2.0.
     * Specify left/right motors, encoder pins, wheel sizes, and track width.
     * Call once in "on start".
     * @param leftMotor left drive motor; eg: motorbit.Motors.M1
     * @param leftPin left encoder pin; eg: DigitalPin.P13
     * @param rightMotor right drive motor; eg: motorbit.Motors.M3
     * @param rightPin right encoder pin; eg: DigitalPin.P14
     */
    //% blockId=motorbit_setup_robot
    //% block="Setup Robot|Left Motor %leftMotor Encoder %leftPin|Right Motor %rightMotor Encoder %rightPin|Wheel Dia L (cm) %leftWheelDia R (cm) %rightWheelDia|Track Width (cm) %trackWidth Ticks/Rev %ticksPerRev"
    //% group="Gorilla Go"
    //% weight=69
    //% leftMotor.defl=motorbit.Motors.M4
    //% leftPin.defl=DigitalPin.P2
    //% rightMotor.defl=motorbit.Motors.M3
    //% rightPin.defl=DigitalPin.P0
    //% leftWheelDia.defl=4.8
    //% rightWheelDia.defl=4.8
    //% trackWidth.defl=8.8
    //% ticksPerRev.defl=270
    //% inlineInputMode=external
    export function setupRobot(
        leftMotor: Motors, leftPin: DigitalPin,
        rightMotor: Motors, rightPin: DigitalPin,
        leftWheelDia: number, rightWheelDia: number,
        trackWidth: number, ticksPerRev: number
    ): void {
        setDriveTrain(leftMotor, leftPin, leftWheelDia, rightMotor, rightPin, rightWheelDia, trackWidth, ticksPerRev);
        initIMU();
        resetYaw();
    }

    /**
     * Set up the arm and gripper servos.
     * Specify which servos to use and their angles.
     * @param liftServo servo for raising/lowering the arm; eg: motorbit.Servos.S1
     * @param liftDownAngle arm angle when lowered (pick position); eg: 30
     * @param liftUpAngle arm angle when raised (carry position); eg: 150
     * @param gripServo servo for the gripper; eg: motorbit.Servos.S2
     * @param gripOpenAngle gripper open angle; eg: 30
     * @param gripCloseAngle gripper close angle (gripping); eg: 110
     */
    //% blockId=motorbit_setup_arm
    //% block="Setup Arm|Lift Servo %liftServo down %liftDownAngle° up %liftUpAngle°|Grip Servo %gripServo open %gripOpenAngle° close %gripCloseAngle°"
    //% group="Gorilla Go"
    //% weight=68
    //% liftServo.defl=motorbit.Servos.S2
    //% gripServo.defl=motorbit.Servos.S1
    //% liftDownAngle.min=0 liftDownAngle.max=180 liftDownAngle.defl=30
    //% liftUpAngle.min=0 liftUpAngle.max=180 liftUpAngle.defl=150
    //% gripOpenAngle.min=0 gripOpenAngle.max=180 gripOpenAngle.defl=30
    //% gripCloseAngle.min=0 gripCloseAngle.max=180 gripCloseAngle.defl=175
    //% inlineInputMode=external
    export function setupArm(
        liftServo: Servos, liftDownAngle: number, liftUpAngle: number,
        gripServo: Servos, gripOpenAngle: number, gripCloseAngle: number
    ): void {
        _arm_liftServo = liftServo;
        _arm_gripServo = gripServo;
        _arm_liftDownAngle = Math.constrain(liftDownAngle, 0, 180);
        _arm_liftUpAngle = Math.constrain(liftUpAngle, 0, 180);
        _arm_gripOpenAngle = Math.constrain(gripOpenAngle, 0, 180);
        _arm_gripCloseAngle = Math.constrain(gripCloseAngle, 0, 180);
    }

    /**
     * Drive straight for a given distance (negative = backward).
     * @param distance distance to travel; eg: 30
     * @param unit cm or inch
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=gorilla_drive_straight
    //% block="Drive Straight %distance %unit at speed %speed"
    //% group="Gorilla Go" weight=99
    //% distance.defl=30
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function driveStraight(distance: number, unit: DistanceUnit, speed: number): void {
        let cm = (unit === DistanceUnit.Inch) ? distance * 2.54 : distance;
        driveDistanceStraight(cm, speed);
    }

    /**
     * Drive straight for a given distance (negative = backward).
     * @param distance distance to travel; eg: 30
     * @param unit cm or inch
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=gorilla_drive_straight
    //% block="Drive Straight %distance %unit at speed %speed"
    //% group="Gorilla Go" weight=98
    //% distance.defl=30 degree.defl=0
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function driveStraightDegree(distance: number, unit: DistanceUnit, speed: number, degree: number): void {
        let cm = (unit === DistanceUnit.Inch) ? distance * 2.54 : distance;
        driveDistanceStraightDegree(cm, speed, degree);
    }

    /**
     * Turn left by a relative angle using tank mode (both wheels move).
     * @param degrees how many degrees to turn left; eg: 90
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=gorilla_turn_left_for
    //% block="Turn Left %degrees ° speed %speed"
    //% group="Gorilla Go" weight=89
    //% degrees.min=0 degrees.max=360 degrees.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnLeftForDegrees(degrees: number, speed: number): void {
        MotorStopAll();
        basic.pause(80);
        turnByYaw(-Math.abs(degrees), speed);
    }

    /**
     * Turn right by a relative angle using tank mode (both wheels move).
     * @param degrees how many degrees to turn right; eg: 90
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=gorilla_turn_right_for
    //% block="Turn Right %degrees ° speed %speed"
    //% group="Gorilla Go" weight=88
    //% degrees.min=0 degrees.max=360 degrees.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnRightForDegrees(degrees: number, speed: number): void {
        MotorStopAll();
        basic.pause(80);
        turnByYaw(Math.abs(degrees), speed);
    }

    /**
     * Turn to face an absolute heading using tank mode (0-360, relative to zero set).
     * @param heading target heading 0-360; eg: 0
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=gorilla_heading_to
    //% block="Heading To %heading ° speed %speed"
    //% group="Gorilla Go" weight=87
    //% heading.min=0 heading.max=360 heading.defl=0
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function headingToDegrees(heading: number, speed: number): void {
        turnToFacingAngle(heading, TurnMode.Tank, speed);
    }

    /**
     * Rotate to face an absolute heading using pivot mode (0-360, relative to zero set).
     * @param heading target heading 0-360; eg: 0
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=gorilla_rotate_to
    //% block="Rotate To %heading ° speed %speed"
    //% group="Gorilla Go" weight=97
    //% heading.min=0 heading.max=360 heading.defl=0
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function rotateToDegrees(heading: number, speed: number): void {
        turnToFacingAngle(heading, TurnMode.Pivot, speed);
    }

    /**
     * Get current heading in degrees 0-360, relative to zero set by setupRobot.
     */
    //% blockId=gorilla_get_degrees
    //% block="Get Degrees (0-360)"
    //% group="Gorilla Go" weight=86
    export function getDegrees(): number {
        let yaw = getRobotYaw();
        if (yaw < 0) yaw += 360;
        return yaw;
    }

    /**
     * Open the gripper.
     */
    //% blockId=motorbit_open_gripper
    //% block="Open Gripper"
    //% group="Gorilla Go"
    //% weight=96
    export function openGripper(): void {
        Servo(_arm_gripServo, _arm_gripOpenAngle);
    }

    /**
     * Close the gripper to grab an object.
     */
    //% blockId=motorbit_close_gripper
    //% block="Close Gripper"
    //% group="Gorilla Go"
    //% weight=95
    export function closeGripper(): void {
        Servo(_arm_gripServo, _arm_gripCloseAngle);
    }

    /**
     * Open the gripper slowly at a given speed.
     * @param speed servo speed 1-10; eg: 5
     */
    //% blockId=motorbit_open_gripper_speed
    //% block="Open Gripper speed %speed"
    //% group="Gorilla Go"
    //% weight=94
    //% speed.min=1 speed.max=10 speed.defl=5
    export function openGripperWithSpeed(speed: number): void {
        Servospeed(_arm_gripServo, _arm_gripCloseAngle, _arm_gripOpenAngle, speed);
    }

    /**
     * Close the gripper slowly at a given speed.
     * @param speed servo speed 1-10; eg: 5
     */
    //% blockId=motorbit_close_gripper_speed
    //% block="Close Gripper speed %speed"
    //% group="Gorilla Go"
    //% weight=93
    //% speed.min=1 speed.max=10 speed.defl=5
    export function closeGripperWithSpeed(speed: number): void {
        Servospeed(_arm_gripServo, _arm_gripOpenAngle, _arm_gripCloseAngle, speed);
    }

    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=motorbit_servo block="Servo|%index|degree|%degree"
    //% group="Servo" weight=100
    //% degree.defl=90
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let v_us = (degree * 1800 / 180 + 600)
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree1 [0-180] degree of servo; eg: 0, 90, 180
     * @param degree2 [0-180] degree of servo; eg: 0, 90, 180
     * @param speed [1-10] speed of servo; eg: 1, 10
    */
    //% blockId=motorbit_servospeed block="Servo|%index|degree start %degree1|end %degree2|speed %speed"
    //% group="Servo" weight=96
    //% degree1.min=0 degree1.max=180
    //% degree2.min=0 degree2.max=180
    //% speed.min=1 speed.max=10
    //% inlineInputMode=inline
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servospeed(index: Servos, degree1: number, degree2: number, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        if (degree1 > degree2) {
            for (let i = degree1; i > degree2; i--) {
                let v_us = (i * 1800 / 180 + 600)
                let value = v_us * 4096 / 20000
                basic.pause(4 * (10 - speed));
                setPwm(index + 7, 0, value)
            }
        }
        else {
            for (let i = degree1; i < degree2; i++) {
                let v_us = (i * 1800 / 180 + 600)
                let value = v_us * 4096 / 20000
                basic.pause(4 * (10 - speed));
                setPwm(index + 7, 0, value)
            }
        }
    }

    //% blockId=motorbit_gservo block="Geek Servo|%index|degree %degree=protractorPicker"
    //% group="GeekServo" weight=96
    //% blockGap=50
    //% degree.defl=90
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let v_us = ((degree - 90) * 20 / 3 + 1500)
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    //% blockId=motorbit_gservo2kg block="GeekServo2KG|%index|degree %degree"
    //% group="GeekServo" weight=95
    //% blockGap=50
    //% degree.min=0 degree.max=360
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo2KG(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let v_us = (Math.floor((degree) * 2000 / 350) + 500)
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    //% blockId=motorbit_gservo5kg block="GeekServo5KG|%index|degree %degree"
    //% group="GeekServo" weight=94
    //% degree.min=0 degree.max=360
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo5KG(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        const minInput = 0;
        const maxInput = 355;
        const minOutput = 500;
        const maxOutput = 2500;
        const v_us = ((degree - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    //% blockId=motorbit_gservo5kg_motor block="GeekServo5KG_MotorEN|%index|speed %speed"
    //% group="GeekServo" weight=93
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo5KG_Motor(index: Servos, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        const minInput = -255;
        const maxInput = 255;
        const minOutput = 5000;
        const maxOutput = 3000;
        const v_us = ((speed - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    //% blockId=motorbit_stepper_degree block="Stepper 28BYJ-48|%index|degree %degree"
    //% group="Stepper Motor" weight=91
    export function StepperDegree(index: Steppers, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(index, degree > 0);
        degree = Math.abs(degree);
        basic.pause(10240 * degree / 360);
        MotorStopAll()
    }

    //% blockId=motorbit_stepper_turn block="Stepper 28BYJ-48|%index|turn %turn"
    //% group="Stepper Motor" weight=90
    export function StepperTurn(index: Steppers, turn: Turns): void {
        let degree = turn;
        StepperDegree(index, degree);
    }

    //% blockId=motorbit_stepper_dual block="Dual Stepper(Degree) |STPM1_2 %degree1| STPM3_4 %degree2"
    //% group="Stepper Motor" weight=89
    export function StepperDual(degree1: number, degree2: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(1, degree1 > 0);
        setStepper(2, degree2 > 0);
        degree1 = Math.abs(degree1);
        degree2 = Math.abs(degree2);
        basic.pause(10240 * Math.min(degree1, degree2) / 360);
        if (degree1 > degree2) {
            stopMotor(3); stopMotor(4);
            basic.pause(10240 * (degree1 - degree2) / 360);
        } else {
            stopMotor(1); stopMotor(2);
            basic.pause(10240 * (degree2 - degree1) / 360);
        }
        MotorStopAll()
    }

    //% blockId=motorbit_stpcar_move block="Car Forward|Distance(cm) %distance|Wheel Diameter(mm) %diameter"
    //% group="Stepper Motor" weight=88
    export function StpCarMove(distance: number, diameter: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * 10 * distance / 3 / diameter;
        setStepper(1, delay > 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    //% blockId=motorbit_stpcar_turn block="Car Turn|Degree %turn|Wheel Diameter(mm) %diameter|Track(mm) %track"
    //% weight=87
    //% group="Stepper Motor" blockGap=50
    export function StpCarTurn(turn: number, diameter: number, track: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * turn * track / 360 / diameter;
        setStepper(1, delay < 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    //% blockId=motorbit_stop_all block="Motor Stop All"
    //% group="Motor" weight=81
    //% blockGap=50
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }

    //% blockId=motorbit_stop block="Motor Stop|%index|"
    //% group="Motor" weight=82
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=motorbit_motor_run block="Motor|%index|speed %speed"
    //% group="Motor" weight=86
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16;
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pp = (index - 1) * 2
        let pn = (index - 1) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }

    //% blockId=motorbit_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
    //% group="Motor" weight=85
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }

    //% blockId=motorbit_motor_dual block="Motor|%motor1|speed %speed1|%motor2|speed %speed2"
    //% group="Motor" weight=84
    //% inlineInputMode=inline
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
    }

    //% blockId=motorbit_motor_dualDelay block="Motor|%motor1|speed %speed1|%motor2|speed %speed2|delay %delay|s "
    //% group="Motor" weight=83
    //% inlineInputMode=inline
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=5
    export function MotorRunDualDelay(motor1: Motors, speed1: number, motor2: Motors, speed2: number, delay: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
        basic.pause(delay * 1000);
        MotorRun(motor1, 0);
        MotorRun(motor2, 0);
    }

    //% blockId="motorbit_rus04" block="On-board Ultrasonic part %index show color %rgb effect %effect"
    //% group="RUS-04" weight=78
    export function motorbit_rus04(index: RgbUltrasonics, rgb: RgbColors, effect: ColorEffect): void {
        sensors.board_rus04_rgb(DigitalPin.P16, 4, index, rgb, effect);
    }

    //% blockId=Ultrasonic_reading_distance block="On-board Ultrasonic reading distance"
    //% group="RUS-04" weight=77
    export function Ultrasonic_reading_distance(): number {
        return sensors.Ultrasonic(DigitalPin.P2);
    }

    //% blockId=Setting_the_on_board_lights block="Setting the on-board lights %index color %rgb"
    //% group="RGB" weight=76
    export function Setting_the_on_board_lights(index: Offset, rgb: RgbColors): void {
        sensors.board_rus04_rgb(DigitalPin.P16, index, 0, rgb, rgb_ColorEffect.None);
    }

    //% blockId=close_the_on_board_lights block="close the on-board lights %index color"
    //% group="RGB" weight=75
    export function close_the_on_board_lights(index: Offset): void {
        sensors.board_rus04_rgb(DigitalPin.P16, index, 0, RgbColors.Black, rgb_ColorEffect.None);
    }

    //% blockId=close_all_the_on_board_lights block="close all the on-board lights"
    //% group="RGB" weight=74
    export function close_all_the_on_board_lights(): void {
        sensors.board_rus04_rgb(DigitalPin.P16, 0, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 1, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 2, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 3, 0, RgbColors.Black, rgb_ColorEffect.None);
    }

    //% blockId=motorbit_set_drivetrain
    //% block="Setup Drive Train|Left: %leftMotor Pin %leftPin Wheel Dia (cm) %leftWheelDia|Right: %rightMotor Pin %rightPin Wheel Dia (cm) %rightWheelDia|Track Width (cm) %trackWidth|Ticks/Rev %ticksPerRev"
    //% group="DriveTrain"
    //% weight=90
    //% leftWheelDia.defl=4.8
    //% rightWheelDia.defl=4.8
    //% trackWidth.defl=8.8
    //% ticksPerRev.defl=270
    //% inlineInputMode=external
    export function setDriveTrain(
        leftMotor: Motors,
        leftPin: DigitalPin,
        leftWheelDia: number,
        rightMotor: Motors,
        rightPin: DigitalPin,
        rightWheelDia: number,
        trackWidth: number,
        ticksPerRev: number
    ): void {
        _dt_leftMotor = leftMotor;
        _dt_rightMotor = rightMotor;
        _dt_leftWheelDia = leftWheelDia;
        _dt_rightWheelDia = rightWheelDia;
        _dt_trackWidth = trackWidth;
        _dt_ticksPerRev = ticksPerRev;

        pins.setPull(leftPin, PinPullMode.PullUp);
        pins.setPull(rightPin, PinPullMode.PullUp);

        _dt_leftCount = 0;
        _dt_rightCount = 0;

        pins.onPulsed(leftPin, PulseValue.High, function () {
            _dt_leftCount++;
        });

        pins.onPulsed(rightPin, PulseValue.High, function () {
            _dt_rightCount++;
        });
    }

    //% blockId=motorbit_drive_distance
    //% block="Drive Distance %cm cm at speed %speed"
    //% group="DriveTrain"
    //% weight=85
    //% speed.min=0 speed.max=255 speed.defl=150
    export function driveDistance(cm: number, speed: number): void {
        const LOOP_MS = 5
        const KP = _tune_driveKp  // gain for cm error between left and right wheels
        const MAX_CORR = 20
        const MIN_SPEED = 20
        const BRAKE_MS = 40
        const TIMEOUT_MS = 8000
        const SLOWDOWN_CM = 4.0  // start slowing down when less than 4 cm remains
        const STOP_EARLY_CM = 1.0
        const STALL_MS = 1500    // stop if encoder counts don't change for this long

        // cm per encoder tick for each wheel
        let leftCmPerTick = (Math.PI * _dt_leftWheelDia) / _dt_ticksPerRev
        let rightCmPerTick = (Math.PI * _dt_rightWheelDia) / _dt_ticksPerRev
        let targetCm = Math.abs(cm)

        _dt_leftCount = 0
        _dt_rightCount = 0

        let dir = cm > 0 ? 1 : -1
        let base = Math.abs(speed)

        kickMotors(base * dir, base * dir)

        let lastCount = -1
        let lastCountMs = control.millis()

        const start = control.millis()
        while (true) {
            let leftCm = _dt_leftCount * leftCmPerTick
            let rightCm = _dt_rightCount * rightCmPerTick
            let avgCm = (leftCm + rightCm) / 2
            let remaining = targetCm - avgCm

            if (remaining <= 0) break
            if (control.millis() - start > TIMEOUT_MS) break

            // stall detection: if encoder is not counting, stop
            let totalCount = _dt_leftCount + _dt_rightCount
            if (totalCount != lastCount) {
                lastCount = totalCount
                lastCountMs = control.millis()
            } else if (control.millis() - lastCountMs > STALL_MS) {
                break
            }

            let curBase = base
            if (remaining <= SLOWDOWN_CM) {
                curBase = MIN_SPEED + Math.round((remaining / SLOWDOWN_CM) * (base - MIN_SPEED))
            }
            if (remaining <= STOP_EARLY_CM) {
                curBase = MIN_SPEED
            }

            // error in real cm (not ticks) — slow down the faster wheel
            let err = leftCm - rightCm
            let corr = Math.round(KP * err)
            if (corr > MAX_CORR) corr = MAX_CORR
            if (corr < -MAX_CORR) corr = -MAX_CORR

            let leftCmd = curBase - corr
            let rightCmd = curBase + corr

            if (leftCmd < MIN_SPEED) leftCmd = MIN_SPEED
            if (rightCmd < MIN_SPEED) rightCmd = MIN_SPEED
            if (leftCmd > 255) leftCmd = 255
            if (rightCmd > 255) rightCmd = 255

            driveMotors(leftCmd * dir, rightCmd * dir)
            basic.pause(LOOP_MS)
        }

        driveMotors(-dir * MIN_SPEED, -dir * MIN_SPEED)
        basic.pause(BRAKE_MS)
        MotorStopAll()
    }
    //% blockId=motorbit_drive_distance_imu
    //% block="Drive Distance (IMU straight) %cm cm at speed %speed"
    //% group="DriveTrain"
    //% weight=84
    //% speed.min=0 speed.max=255 speed.defl=150
    export function driveDistanceStraight(cm: number, speed: number): void {
        // record heading at start — use IMU if available
        let startYaw = _imu_initialized ? getRobotYaw() : 0
        driveDistanceStraightDegree(cm, speed, startYaw)
    }


    //% blockId=motorbit_drive_distance_imu_degree
    //% block="Drive Distance (IMU straight) %cm cm at speed %speed at angle %startYaw"
    //% group="DriveTrain"
    //% weight=83
    //% speed.min=0 speed.max=255 speed.defl=150 startYaw.defl=0
    export function driveDistanceStraightDegree(cm: number, speed: number, startYaw: number): void {
        const LOOP_MS = 5
        const KP_ENC = _tune_driveKp     // gain from encoder (cm difference between left and right)
        const KP_HDG = _tune_driveHdgKp  // gain from IMU heading (degrees drifted)
        const MAX_CORR = 35
        const MIN_SPEED = 20
        const BRAKE_MS = 40
        const TIMEOUT_MS = 8000
        const SLOWDOWN_CM = 4.0
        const STOP_EARLY_CM = 1.0
        const STALL_MS = 1500    // stop if encoder counts don't change for this long

        let leftCmPerTick = (Math.PI * _dt_leftWheelDia) / _dt_ticksPerRev
        let rightCmPerTick = (Math.PI * _dt_rightWheelDia) / _dt_ticksPerRev
        let targetCm = Math.abs(cm)

        _dt_leftCount = 0
        _dt_rightCount = 0

        let dir = cm > 0 ? 1 : -1
        let base = Math.abs(speed)

        kickMotors(base * dir, base * dir)

        let lastCount = -1
        let lastCountMs = control.millis()

        const start = control.millis()
        while (true) {
            let leftCm = _dt_leftCount * leftCmPerTick
            let rightCm = _dt_rightCount * rightCmPerTick
            let avgCm = (leftCm + rightCm) / 2
            let remaining = targetCm - avgCm

            if (remaining <= 0) break
            if (control.millis() - start > TIMEOUT_MS) break

            // stall detection: if encoder is not counting, stop
            let totalCount = _dt_leftCount + _dt_rightCount
            if (totalCount != lastCount) {
                lastCount = totalCount
                lastCountMs = control.millis()
            } else if (control.millis() - lastCountMs > STALL_MS) {
                break
            }

            let curBase = base
            if (remaining <= SLOWDOWN_CM) {
                curBase = MIN_SPEED + Math.round((remaining / SLOWDOWN_CM) * (base - MIN_SPEED))
            }
            if (remaining <= STOP_EARLY_CM) {
                curBase = MIN_SPEED
            }

            // encoder correction: slow down whichever wheel has gone farther
            // (leftCm - rightCm) < 0 when right > left, giving corr < 0 → left gets more speed → correct for both forward and backward
            let corr = Math.round(KP_ENC * (leftCm - rightCm))

            // IMU heading correction: multiply by dir so correction flips correctly when going backward
            if (_imu_initialized) {
                let hdgErr = getRobotYaw() - startYaw
                while (hdgErr > 180) hdgErr -= 360
                while (hdgErr < -180) hdgErr += 360
                // hdgErr > 0 = drifted CW → corr += negative when backward (dir=-1) → left gets more → CCW
                corr += Math.round(KP_HDG * hdgErr * _dt_imuInvert * dir)
            }

            if (corr > MAX_CORR) corr = MAX_CORR
            if (corr < -MAX_CORR) corr = -MAX_CORR

            let leftCmd = curBase - corr
            let rightCmd = curBase + corr

            if (leftCmd < MIN_SPEED) leftCmd = MIN_SPEED
            if (rightCmd < MIN_SPEED) rightCmd = MIN_SPEED
            if (leftCmd > 255) leftCmd = 255
            if (rightCmd > 255) rightCmd = 255

            driveMotors(leftCmd * dir, rightCmd * dir)
            basic.pause(LOOP_MS)
        }

        driveMotors(-dir * MIN_SPEED, -dir * MIN_SPEED)
        basic.pause(BRAKE_MS)
        MotorStopAll()
    }

    //% blockId=motorbit_tank_turn
    //% block="Tank Turn %degree ° at speed %speed"
    //% group="DriveTrain"
    //% weight=80
    //% speed.min=0 speed.max=255 speed.defl=150
    export function tankTurn(degree: number, speed: number): void {
        if (degree == 0) return

        let base = Math.abs(speed)
        if (base < 30) base = 30
        if (base > 255) base = 255

        let turnCirc = Math.PI * _dt_trackWidth
        let dist = (Math.abs(degree) / 360) * turnCirc
        let avgWheelDia = (_dt_leftWheelDia + _dt_rightWheelDia) / 2
        let wheelCirc = Math.PI * avgWheelDia
        let targetTicks = (dist / wheelCirc) * _dt_ticksPerRev

        let turnK = (typeof _dt_turnK === "number") ? _dt_turnK : 1.00
        targetTicks = targetTicks * turnK

        _dt_leftCount = 0
        _dt_rightCount = 0

        // degree > 0 = turn right: left wheel forward, right wheel backward
        let leftDir = (degree > 0) ? +1 : -1
        let rightDir = -leftDir

        const KP = _tune_encTurnKp
        const MAX_CORR = 40
        const MIN_CMD = 40
        const LOOP_MS = 5
        const TIMEOUT_MS = 5000

        let t0 = control.millis()

        driveMotors(leftDir * base, rightDir * base)

        while (true) {
            let l = Math.abs(_dt_leftCount)
            let r = Math.abs(_dt_rightCount)
            let avg = (l + r) / 2

            if (avg >= targetTicks) break
            if (control.millis() - t0 > TIMEOUT_MS) break

            let err = l - r
            let corr = Math.round(KP * err)
            if (corr > MAX_CORR) corr = MAX_CORR
            if (corr < -MAX_CORR) corr = -MAX_CORR

            let leftCmd = base - corr
            let rightCmd = base + corr

            if (leftCmd < MIN_CMD) leftCmd = MIN_CMD
            if (rightCmd < MIN_CMD) rightCmd = MIN_CMD
            if (leftCmd > 255) leftCmd = 255
            if (rightCmd > 255) rightCmd = 255

            driveMotors(leftDir * leftCmd, rightDir * rightCmd)

            basic.pause(LOOP_MS)
        }

        MotorStopAll()
    }

    //% blockId=motorbit_pivot_turn
    //% block="Pivot Turn %degree ° at speed %speed"
    //% group="DriveTrain"
    //% weight=79
    //% speed.min=0 speed.max=255 speed.defl=150
    export function pivotTurn(degree: number, speed: number): void {
        if (degree == 0) return

        let base = Math.abs(speed)
        if (base < 40) base = 40
        if (base > 255) base = 255

        _dt_leftCount = 0
        _dt_rightCount = 0

        const LOOP_MS = 5
        const TIMEOUT_MS = 4000
        const MIN_SPEED = 30

        let t0 = control.millis()

        if (degree > 0) {
            // turn right: left wheel spins (uses leftWheelDia), right wheel stops
            let targetTicks = (Math.abs(degree) / 360) * (2 * _dt_trackWidth / _dt_leftWheelDia) * _dt_ticksPerRev * _dt_turnK
            let slowZone = targetTicks * 0.3
            MotorStop(_dt_rightMotor)

            while (true) {
                let ticks = Math.abs(_dt_leftCount)
                let remain = targetTicks - ticks

                if (remain <= 0) break
                if (control.millis() - t0 > TIMEOUT_MS) break

                let cmd = base
                if (remain < slowZone) {
                    cmd = Math.map(remain, 0, slowZone, MIN_SPEED, base)
                    if (cmd < MIN_SPEED) cmd = MIN_SPEED
                }

                driveMotors(cmd, 0)
                basic.pause(LOOP_MS)
            }

        } else {
            // turn left: right wheel spins (uses rightWheelDia), left wheel stops
            let targetTicks = (Math.abs(degree) / 360) * (2 * _dt_trackWidth / _dt_rightWheelDia) * _dt_ticksPerRev * _dt_turnK
            let slowZone = targetTicks * 0.3
            MotorStop(_dt_leftMotor)

            while (true) {
                let ticks = Math.abs(_dt_rightCount)
                let remain = targetTicks - ticks

                if (remain <= 0) break
                if (control.millis() - t0 > TIMEOUT_MS) break

                let cmd = base
                if (remain < slowZone) {
                    cmd = Math.map(remain, 0, slowZone, MIN_SPEED, base)
                    if (cmd < MIN_SPEED) cmd = MIN_SPEED
                }

                driveMotors(0, cmd)
                basic.pause(LOOP_MS)
            }
        }

        MotorStopAll()
    }

    function bnoWrite(reg: number, value: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(BNO055_ADDR, buf);
    }

    function bnoRead(reg: number): number {
        pins.i2cWriteNumber(BNO055_ADDR, reg, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(BNO055_ADDR, NumberFormat.UInt8BE);
    }

    //% blockId=motorbit_init_imu
    //% block="Initialize IMU"
    //% group="IMU"
    //% weight=60
    export function initIMU(): void {
        let id = bnoRead(BNO_REG_CHIP_ID);
        if (id != 0xA0) {
            return;
        }
        bnoWrite(BNO_REG_OPR_MODE, OP_MODE_CONFIG);
        basic.pause(20);
        bnoWrite(BNO_REG_UNIT_SEL, 0x00);
        basic.pause(10);
        bnoWrite(BNO_REG_OPR_MODE, OP_MODE_IMU);
        basic.pause(20);
        _imu_initialized = true;
    }

    //% blockId=motorbit_get_heading
    //% block="Get Heading (0-360)"
    //% group="IMU"
    //% weight=59
    export function getHeading(): number {
        if (!_imu_initialized) return 0;
        pins.i2cWriteNumber(BNO055_ADDR, BNO_REG_EUL_HEADING_LSB, NumberFormat.UInt8BE);
        let buf = pins.i2cReadBuffer(BNO055_ADDR, 2);
        let headingRaw = (buf[1] << 8) | buf[0];
        let heading = headingRaw / 16.0;
        return heading;
    }

    //% blockId=motorbit_get_robot_yaw
    //% block="Get Robot Yaw (-180 to 180)"
    //% group="IMU"
    //% weight=58
    export function getRobotYaw(): number {
        let current = getHeading();
        let relative = current - _imu_heading_offset;
        while (relative > 180) relative -= 360;
        while (relative < -180) relative += 360;
        return relative;
    }

    //% blockId=motorbit_reset_yaw
    //% block="Reset Yaw to 0"
    //% group="Gorilla Go"
    //% weight=92
    export function resetYaw(): void {
        _imu_heading_offset = getHeading();
    }

    //% blockId=motorbit_get_calib_status
    //% block="Get IMU Calibration Status (0-3)"
    //% group="IMU"
    //% weight=55
    //% advanced=true
    export function getCalibrationStatus(): number {
        if (!_imu_initialized) return 0;
        let calib = bnoRead(BNO_REG_CALIB_STAT);
        return (calib >> 6) & 0x03;
    }

    //% blockId=motorbit_turn_to_yaw_best2
    //% block="Turn to Yaw (best2) %targetYaw deg speed %maxSpeed loop %ms ms"
    //% group="DriveTrain"
    //% weight=78
    //% maxSpeed.min=0 maxSpeed.max=255 maxSpeed.defl=110
    //% ms.min=5 ms.max=50 ms.defl=15
    export function turnToYaw(targetYaw: number, maxSpeed: number, ms: number): void {
        if (!_imu_initialized) return
        if (ms <= 0) ms = 15

        const TIMEOUT_MS = 9000
        const MIN_CMD = 20
        const MAX_CMD = Math.min(105, Math.abs(maxSpeed))
        const KP = _tune_imuTurnKp
        const KD = _tune_imuTurnKd
        const TOL = 1
        const SETTLE_N = 6
        const SLOW_ZONE = 50
        const SLOW_CMD = _tune_trimSlowCmd
        const TRIM_ZONE = 10
        const TRIM_CMD = _tune_trimCmd
        const PULSE_MIN_MS = _tune_trimPulseMin
        const PULSE_MAX_MS = _tune_trimPulseMax
        const STOP_BETWEEN_MS = 70
        const STUCK_DEG = 0.2
        const STUCK_LIMIT = 2
        const STUCK_GIVE_UP = 3
        const DECEL_MS = _tune_trimDecelMs

        function norm180(a: number): number {
            while (a > 180) a -= 360
            while (a < -180) a += 360
            return a
        }

        targetYaw = norm180(targetYaw)

        let initErr0 = norm180(targetYaw - getRobotYaw())
        let initDir0 = initErr0 >= 0 ? 1 : -1
        let initCmd0 = Math.round(KP * Math.abs(initErr0))
        if (initCmd0 < MIN_CMD) initCmd0 = MIN_CMD
        if (initCmd0 > MAX_CMD) initCmd0 = MAX_CMD
        kickMotors(initDir0 * initCmd0, -initDir0 * initCmd0)

        let t0 = control.millis()
        let settle = 0
        let lastErr = norm180(targetYaw - getRobotYaw())
        let stuckBonus = 0
        let stuckCnt = 0
        let stuckGiveUpCnt = 0
        let lastYaw = getRobotYaw()
        let done = false
        let firstTrim = true

        while (!done) {
            if (control.millis() - t0 > TIMEOUT_MS) break

            let yaw = getRobotYaw()
            let err = norm180(targetYaw - yaw)
            let aerr = Math.abs(err)
            let dir = (err >= 0) ? 1 : -1

            if (aerr <= TOL) {
                settle++
                MotorStopAll()
                basic.pause(ms)
                if (settle >= SETTLE_N) break
                continue
            } else {
                settle = 0
            }

            // Near zone: micro-pulse trim
            if (aerr <= TRIM_ZONE) {
                // First entry: stop and wait for robot to fully decelerate
                if (firstTrim) {
                    MotorStopAll()
                    basic.pause(DECEL_MS)
                    lastYaw = getRobotYaw()
                    firstTrim = false
                    continue
                }
                let want = PULSE_MIN_MS + Math.round((aerr / TRIM_ZONE) * (PULSE_MAX_MS - PULSE_MIN_MS))
                let usePulse = want + stuckBonus
                driveMotors(dir * TRIM_CMD, -dir * TRIM_CMD)
                basic.pause(usePulse)
                MotorStopAll()
                basic.pause(STOP_BETWEEN_MS)

                let yaw2 = getRobotYaw()
                let dy = Math.abs(norm180(yaw2 - lastYaw))
                lastYaw = yaw2

                if (dy < STUCK_DEG) {
                    stuckCnt++
                    if (stuckCnt >= STUCK_LIMIT) {
                        stuckBonus += 5
                        if (stuckBonus > 30) stuckBonus = 30
                        stuckCnt = 0
                        stuckGiveUpCnt++
                        if (stuckGiveUpCnt >= STUCK_GIVE_UP) done = true
                    }
                } else {
                    stuckCnt = 0
                    stuckGiveUpCnt = 0
                    if (stuckBonus > 0) stuckBonus -= 2
                }

                continue
            }

            // Far zone: PD coarse — derr in deg/iteration (not per-second, avoids KD dominating)
            let derr = err - lastErr
            lastErr = err

            let u = KP * aerr - KD * Math.abs(derr)

            let cmd = Math.round(u)
            if (cmd < MIN_CMD) cmd = MIN_CMD
            if (cmd > MAX_CMD) cmd = MAX_CMD

            // Slow zone: linearly ramp speed from SLOW_CMD down to MIN_CMD as aerr → TRIM_ZONE
            if (aerr <= SLOW_ZONE) {
                let ramp = MIN_CMD + Math.round(((aerr - TRIM_ZONE) / (SLOW_ZONE - TRIM_ZONE)) * (SLOW_CMD - MIN_CMD))
                if (ramp < MIN_CMD) ramp = MIN_CMD
                if (ramp > SLOW_CMD) ramp = SLOW_CMD
                if (cmd > ramp) cmd = ramp
            }

            driveMotors(dir * cmd, -dir * cmd)

            basic.pause(ms)
        }

        MotorStopAll()
    }

    //% blockId=motorbit_turn_by_yaw
    //% block="Turn by Yaw %deltaYaw degrees at speed %maxSpeed"
    //% group="DriveTrain"
    //% weight=77
    //% maxSpeed.min=0 maxSpeed.max=255 maxSpeed.defl=120
    export function turnByYaw(deltaYaw: number, maxSpeed: number): void {
        let target = getRobotYaw() + deltaYaw
        while (target > 180) target -= 360
        while (target < -180) target += 360
        turnToYaw(target, maxSpeed, 5)
    }

    // ==========================================
    // Robot Setup
    // ==========================================
    let _arm_liftServo: Servos = Servos.S1;
    let _arm_gripServo: Servos = Servos.S2;
    let _arm_liftDownAngle: number = 30;
    let _arm_liftUpAngle: number = 150;
    let _arm_gripOpenAngle: number = 30;
    let _arm_gripCloseAngle: number = 110;

    /**
     * Set motor direction for each side.
     * If the robot drives backward when it should go forward, set leftForward or rightForward to false.
     * Default: left = true, right = false (right motor is physically mounted reversed).
     * @param leftForward true = left motor forward is positive; eg: true
     * @param rightForward true = right motor forward is positive; eg: false
     */
    //% blockId=motorbit_set_motor_direction
    //% block="Set Motor Direction|Left forward %leftForward|Right forward %rightForward"
    //% group="Robot Setup"
    //% weight=98
    //% leftForward.defl=true
    //% rightForward.defl=false
    //% inlineInputMode=external
    export function setMotorDirection(leftForward: boolean, rightForward: boolean): void {
        _dt_leftInvert = leftForward ? 1 : -1;
        _dt_rightInvert = rightForward ? 1 : -1;
    }

    /**
     * Invert the IMU yaw direction.
     * Set to true if the robot curves the wrong way during IMU-guided straight driving or turns.
     * This happens when the BNO055 is mounted upside-down or rotated 180°.
     * @param inverted true = invert yaw correction; eg: false
     */
    //% blockId=motorbit_set_imu_invert
    //% block="Invert IMU Direction %inverted"
    //% group="Robot Setup"
    //% weight=97
    //% inverted.defl=false
    export function setIMUInvert(inverted: boolean): void {
        _dt_imuInvert = inverted ? -1 : 1;
    }

    /**
     * Tune the drive-straight PID gains.
     * Increase driveKp if the robot drifts sideways. Increase hdgKp if heading drifts (IMU mode).
     * @param driveKp encoder balance gain (default 1.5); eg: 1.5
     * @param hdgKp IMU heading correction gain (default 3.5); eg: 3.5
     */
    //% blockId=motorbit_tune_drive_pid
    //% block="Tune Drive PID|encoder balance KP %driveKp|IMU heading KP %hdgKp"
    //% group="Robot Setup"
    //% weight=97
    //% driveKp.defl=1.5
    //% hdgKp.defl=3.5
    //% inlineInputMode=external
    export function tuneDrivePID(driveKp: number, hdgKp: number): void {
        _tune_driveKp = driveKp;
        _tune_driveHdgKp = hdgKp;
    }

    /**
     * Tune the IMU turn PID gains.
     * Increase KP to turn faster. Increase KD to reduce overshoot.
     * @param kp proportional gain (default 2.2); eg: 2.2
     * @param kd derivative gain (default 8.0); eg: 8.0
     */
    //% blockId=motorbit_tune_imu_turn_pid
    //% block="Tune IMU Turn PID|KP %kp|KD %kd"
    //% group="Robot Setup"
    //% weight=96
    //% kp.defl=2.2
    //% kd.defl=8.0
    //% inlineInputMode=inline
    export function tuneIMUTurnPID(kp: number, kd: number): void {
        _tune_imuTurnKp = kp;
        _tune_imuTurnKd = kd;
    }

    /**
     * Tune the encoder turn parameters.
     * Increase scale if turns are too short; decrease if too long.
     * Increase KP if one wheel leads the other.
     * @param encKp encoder balance gain (default 0.6); eg: 0.6
     * @param scale turn distance scale factor (default 1.0); eg: 1.0
     */
    //% blockId=motorbit_tune_enc_turn
    //% block="Tune Encoder Turn|balance KP %encKp|scale %scale"
    //% group="Robot Setup"
    //% weight=95
    //% encKp.defl=0.6
    //% scale.defl=1.0
    //% inlineInputMode=inline
    export function tuneEncoderTurn(encKp: number, scale: number): void {
        _tune_encTurnKp = encKp;
        _dt_turnK = scale;
    }

    /**
     * Tune the IMU turn trim zone parameters.
     * slowCmd: top speed in slow approach zone (lower = less overshoot, default 32).
     * decelMs: pause (ms) before first micro-pulse to let robot stop (default 150).
     * trimCmd: motor power for micro-pulses (lower = finer steps, default 70).
     * pulseMin/Max: micro-pulse duration range in ms (default 3-5).
     * @param slowCmd max speed in slow approach zone; eg: 32
     * @param decelMs ms to wait before first micro-pulse; eg: 150
     * @param trimCmd motor command for micro-pulses; eg: 70
     * @param pulseMin minimum pulse duration ms; eg: 3
     * @param pulseMax maximum pulse duration ms; eg: 5
     */
    //% blockId=motorbit_tune_turn_trim
    //% block="Tune Turn Trim|Slow speed %slowCmd|Decel wait (ms) %decelMs|Pulse power %trimCmd|Pulse min (ms) %pulseMin max (ms) %pulseMax"
    //% group="Robot Setup"
    //% weight=94
    //% slowCmd.min=20 slowCmd.max=80 slowCmd.defl=32
    //% decelMs.min=50 decelMs.max=400 decelMs.defl=150
    //% trimCmd.min=40 trimCmd.max=100 trimCmd.defl=70
    //% pulseMin.min=1 pulseMin.max=20 pulseMin.defl=3
    //% pulseMax.min=1 pulseMax.max=30 pulseMax.defl=5
    //% inlineInputMode=external
    export function tuneTurnTrim(slowCmd: number, decelMs: number, trimCmd: number, pulseMin: number, pulseMax: number): void {
        _tune_trimSlowCmd = slowCmd;
        _tune_trimDecelMs = decelMs;
        _tune_trimCmd = trimCmd;
        _tune_trimPulseMin = pulseMin;
        _tune_trimPulseMax = Math.max(pulseMax, pulseMin);
    }

    // ==========================================
    // Robot Drive
    // ==========================================

    /**
     * Drive forward continuously until stopRobot is called.
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_drive_forward
    //% block="Drive Forward speed %speed"
    //% group="Robot Drive"
    //% weight=95
    //% speed.min=0 speed.max=255 speed.defl=150
    export function driveForward(speed: number): void {
        if (!initialized) initPCA9685();
        speed = Math.constrain(speed, 0, 255);
        driveMotors(speed, speed);
    }

    /**
     * Drive backward continuously until stopRobot is called.
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_drive_backward
    //% block="Drive Backward speed %speed"
    //% group="Robot Drive"
    //% weight=94
    //% speed.min=0 speed.max=255 speed.defl=150
    export function driveBackward(speed: number): void {
        if (!initialized) initPCA9685();
        speed = Math.constrain(speed, 0, 255);
        driveMotors(-speed, -speed);
    }

    /**
     * Spin left in place (tank turn) continuously until stopRobot is called.
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_robot_spin_left
    //% block="Spin Left speed %speed"
    //% group="Robot Drive"
    //% weight=93
    //% speed.min=0 speed.max=255 speed.defl=150
    export function spinLeft(speed: number): void {
        if (!initialized) initPCA9685();
        speed = Math.constrain(speed, 0, 255);
        driveMotors(-speed, speed);
    }

    /**
     * Spin right in place (tank turn) continuously until stopRobot is called.
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_robot_spin_right
    //% block="Spin Right speed %speed"
    //% group="Robot Drive"
    //% weight=92
    //% speed.min=0 speed.max=255 speed.defl=150
    export function spinRight(speed: number): void {
        if (!initialized) initPCA9685();
        speed = Math.constrain(speed, 0, 255);
        driveMotors(speed, -speed);
    }

    /**
     * Stop all motors.
     */
    //% blockId=motorbit_stop_robot
    //% block="Stop Robot"
    //% group="Robot Drive"
    //% weight=91
    export function stopRobot(): void {
        MotorStopAll();
    }

    /**
     * Drive a set distance using encoders. Choose cm or inch.
     * Call setupRobot first.
     * @param distance distance to travel; eg: 30
     * @param unit cm or inch
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_drive_distance_unit
    //% block="Drive %distance %unit at speed %speed"
    //% group="Robot Drive"
    //% weight=88
    //% distance.defl=30
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function driveDistanceTo(distance: number, unit: DistanceUnit, speed: number): void {
        let cm = (unit === DistanceUnit.Inch) ? distance * 2.54 : distance;
        driveDistance(cm, speed);
    }

    /**
     * Drive straight using both encoders (distance) and IMU (heading correction).
     * Call first.
     * @param distance distance to travel; eg: 30
     * @param unit cm or inch
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_drive_straight_imu
    //% block="Drive Straight %distance %unit at speed %speed (IMU)"
    //% group="Robot Drive"
    //% weight=87
    //% distance.defl=30
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function driveStraightTo(distance: number, unit: DistanceUnit, speed: number): void {
        let cm = (unit === DistanceUnit.Inch) ? distance * 2.54 : distance;
        driveDistanceStraight(cm, speed);
    }

    /**
     * Turn right by N degrees using the IMU.
     * Call setupRobot first.
     * @param angle degrees to turn (0-360); eg: 90
     */
    //% blockId=motorbit_turn_right_imu
    //% block="Turn Right %angle degrees (IMU)"
    //% group="Robot Drive"
    //% weight=90
    //% angle.min=0 angle.max=360 angle.defl=90
    export function turnRightDegrees(angle: number): void {
        turnByYaw(Math.abs(angle), 120);
    }

    /**
     * Turn left by N degrees using the IMU.
     * Call setupRobot first.
     * @param angle degrees to turn (0-360); eg: 90
     */
    //% blockId=motorbit_turn_left_imu
    //% block="Turn Left %angle degrees (IMU)"
    //% group="Robot Drive"
    //% weight=89
    //% angle.min=0 angle.max=360 angle.defl=90
    export function turnLeftDegrees(angle: number): void {
        turnByYaw(-Math.abs(angle), 120);
    }

    /**
     * Turn by a signed angle using the IMU.
     * Positive = right, negative = left. Range: -180 to 180.
     * Call setupRobot first.
     * @param angle angle in degrees -180 to 180; eg: 90
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=motorbit_turn_by_signed
    //% block="Turn by %angle ° (±180) speed %speed"
    //% group="Robot Drive"
    //% weight=87
    //% angle.min=-180 angle.max=180 angle.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnBy(angle: number, speed: number): void {
        angle = Math.constrain(angle, -180, 180);
        turnByYaw(angle, speed);
    }

    /**
     * Turn to face an absolute heading using the IMU.
     * 0=forward, 90=right, 180=back, 270=left (relative to resetYaw direction).
     * Call setupRobot first.
     * @param heading target heading 0-360 degrees; eg: 90
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=motorbit_turn_to_facing
    //% block="Turn to face %heading ° (0-360) speed %speed"
    //% group="Robot Drive"
    //% weight=86
    //% heading.min=0 heading.max=360 heading.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnToFacing(heading: number, speed: number): void {
        // normalise 0-360 → -180..180 to match internal yaw system
        heading = heading % 360;
        if (heading < 0) heading += 360;
        let yaw = heading > 180 ? heading - 360 : heading;
        turnToYaw(yaw, speed, 15);
    }

    // IMU pivot turn — internal helper, one wheel spins, the other stays still
    function turnToYawPivot(targetYaw: number, maxSpeed: number): void {
        if (!_imu_initialized) return;

        const TIMEOUT_MS = 9000;
        const TOL = 3;
        const SETTLE_N = 8;
        const MIN_CMD = 20;
        const MAX_CMD = Math.min(120, Math.abs(maxSpeed));
        const KP = _tune_imuTurnKp;
        const LOOP_MS = 10;
        const TRIM_ZONE = 12;
        const TRIM_CMD = 75;
        const PULSE_MIN_MS = 10;
        const PULSE_MAX_MS = 22;
        const STOP_BETWEEN_MS = 40;

        function norm180p(a: number): number {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }

        targetYaw = norm180p(targetYaw);

        let initErr0p = norm180p(targetYaw - getRobotYaw());
        let initCmd0p = Math.round(KP * Math.abs(initErr0p));
        if (initCmd0p < MIN_CMD) initCmd0p = MIN_CMD;
        if (initCmd0p > MAX_CMD) initCmd0p = MAX_CMD;
        if (initErr0p > 0) {
            kickMotors(initCmd0p, 0);
        } else if (initErr0p < 0) {
            kickMotors(0, initCmd0p);
        }

        let t0 = control.millis();
        let settle = 0;
        let stuckBonus = 0;
        let lastYaw = getRobotYaw();
        let stuckCnt = 0;

        while (true) {
            if (control.millis() - t0 > TIMEOUT_MS) break;

            let err = norm180p(targetYaw - getRobotYaw());
            let aerr = Math.abs(err);

            if (aerr <= TOL) {
                settle++;
                MotorStopAll();
                basic.pause(LOOP_MS);
                if (settle >= SETTLE_N) break;
                continue;
            }
            settle = 0;

            // Near target: micro-pulse to avoid overshoot
            if (aerr <= TRIM_ZONE) {
                let want = PULSE_MIN_MS + Math.round((aerr / TRIM_ZONE) * (PULSE_MAX_MS - PULSE_MIN_MS));
                let usePulse = want + stuckBonus;
                if (err > 0) {
                    driveMotors(TRIM_CMD, 0);
                } else {
                    driveMotors(0, TRIM_CMD);
                }
                basic.pause(usePulse);
                MotorStopAll();
                basic.pause(STOP_BETWEEN_MS);

                let yaw2 = getRobotYaw();
                let dy = Math.abs(norm180p(yaw2 - lastYaw));
                lastYaw = yaw2;
                if (dy < 0.25) {
                    stuckCnt++;
                    if (stuckCnt >= 2) {
                        stuckBonus += 5;
                        if (stuckBonus > 30) stuckBonus = 30;
                        stuckCnt = 0;
                    }
                } else {
                    stuckCnt = 0;
                    if (stuckBonus > 0) stuckBonus -= 2;
                }
                continue;
            }

            let cmd = Math.round(KP * aerr);
            if (cmd < MIN_CMD) cmd = MIN_CMD;
            if (cmd > MAX_CMD) cmd = MAX_CMD;

            // err > 0 = turn right: left wheel spins, right wheel stops
            // err < 0 = turn left: right wheel spins, left wheel stops
            if (err > 0) {
                driveMotors(cmd, 0);
            } else {
                driveMotors(0, cmd);
            }
            basic.pause(LOOP_MS);
        }
        MotorStopAll();
    }

    /**
     * Turn by a relative angle using encoders.
     * Choose Tank (2 motors) or Pivot (1 motor) mode.
     * Positive = right, negative = left. Range: -180 to 180.
     * @param angle angle in degrees -180 to 180; eg: 90
     * @param mode turn mode (Tank or Pivot)
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=motorbit_turn_by_angle_mode
    //% block="Turn by %angle ° (±180) %mode speed %speed"
    //% group="Robot Drive"
    //% weight=85
    //% angle.min=-180 angle.max=180 angle.defl=90
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function turnByAngle(angle: number, mode: TurnMode, speed: number): void {
        if (mode === TurnMode.Pivot) {
            pivotTurn(angle, speed);
        } else {
            tankTurn(angle, speed);
        }
    }

    /**
     * Turn to face an absolute heading using the IMU.
     * Choose Tank (2 motors) or Pivot (1 motor) mode.
     * 0=forward, 90=right, 180=back, 270=left.
     * @param heading target heading 0-360; eg: 90
     * @param mode turn mode (Tank or Pivot)
     * @param speed motor speed 0-255; eg: 120
     */
    //% blockId=motorbit_turn_to_facing_mode
    //% block="Turn to face %heading ° (0-360) %mode speed %speed"
    //% group="Robot Drive"
    //% weight=84
    //% heading.min=0 heading.max=360 heading.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnToFacingAngle(heading: number, mode: TurnMode, speed: number): void {
        heading = heading % 360;
        if (heading < 0) heading += 360;
        let yaw = heading > 180 ? heading - 360 : heading;
        if (mode === TurnMode.Pivot) {
            turnToYawPivot(yaw, speed);
        } else {
            turnToYaw(yaw, speed, 15);
        }
    }

    // ==========================================
    // Robot Arm
    // ==========================================

    /**
     * Raise the arm to the carry position.
     */
    //% blockId=motorbit_lift_arm
    //% block="Lift Arm Up"
    //% group="Robot Arm"
    //% weight=88
    export function liftArm(): void {
        Servo(_arm_liftServo, _arm_liftUpAngle);
    }

    /**
     * Lower the arm to the pick-up position.
     */
    //% blockId=motorbit_lower_arm
    //% block="Lower Arm Down"
    //% group="Robot Arm"
    //% weight=87
    export function lowerArm(): void {
        Servo(_arm_liftServo, _arm_liftDownAngle);
    }

    /**
     * Pick up an object: lower arm → open gripper → close gripper → lift arm.
     */
    //% blockId=motorbit_pick_up
    //% block="Pick Up Object"
    //% group="Robot Arm"
    //% weight=84
    export function pickUpObject(): void {
        lowerArm();
        basic.pause(400);
        openGripper();
        basic.pause(400);
        closeGripper();
        basic.pause(500);
        liftArm();
        basic.pause(400);
    }

    /**
     * Release an object: lower arm → open gripper → lift arm.
     */
    //% blockId=motorbit_release_object
    //% block="Release Object"
    //% group="Robot Arm"
    //% weight=83
    export function releaseObject(): void {
        lowerArm();
        basic.pause(400);
        openGripper();
        basic.pause(500);
        liftArm();
        basic.pause(400);
    }

    /**
     * Set the arm servo to a custom angle.
     * @param angle angle 0-180 degrees; eg: 90
     */
    //% blockId=motorbit_set_arm_angle
    //% block="Set Arm angle %angle °"
    //% group="Robot Arm"
    //% weight=82
    //% angle.min=0 angle.max=180 angle.defl=90
    export function setArmAngle(angle: number): void {
        Servo(_arm_liftServo, Math.constrain(angle, 0, 180));
    }

    /**
     * Set the gripper servo to a custom angle.
     * @param angle angle 0-180 degrees; eg: 90
     */
    //% blockId=motorbit_set_gripper_angle
    //% block="Set Gripper angle %angle °"
    //% group="Robot Arm"
    //% weight=81
    //% angle.min=0 angle.max=180 angle.defl=90
    export function setGripperAngle(angle: number): void {
        Servo(_arm_gripServo, Math.constrain(angle, 0, 180));
    }

}