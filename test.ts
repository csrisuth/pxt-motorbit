// tests go here; this will not be compiled when this package is used as a library
// motorbit.Setting_the_on_board_lights(Offset.ONE, RgbColors.Red)
// motorbit.Setting_the_on_board_lights(Offset.TWO, RgbColors.Red)
// motorbit.Setting_the_on_board_lights(Offset.THREE, RgbColors.Red)
// motorbit.Setting_the_on_board_lights(Offset.FOUR, RgbColors.Red)
// basic.pause(100)
// motorbit.Setting_the_on_board_lights(Offset.ONE, RgbColors.Green)
// motorbit.Setting_the_on_board_lights(Offset.TWO, RgbColors.Green)
// motorbit.Setting_the_on_board_lights(Offset.THREE, RgbColors.Green)
// motorbit.Setting_the_on_board_lights(Offset.FOUR, RgbColors.Green)
// basic.pause(100)
// motorbit.Setting_the_on_board_lights(Offset.ONE, RgbColors.Blue)
// motorbit.Setting_the_on_board_lights(Offset.TWO, RgbColors.Blue)
// motorbit.Setting_the_on_board_lights(Offset.THREE, RgbColors.Blue)
// motorbit.Setting_the_on_board_lights(Offset.FOUR, RgbColors.Blue)
// basic.pause(100)
// motorbit.close_all_the_on_board_lights()
// music.playTone(262, music.beat(BeatFraction.Whole))
// basic.pause(100)
// music.stopAllSounds()

motorbit.setupRobot(
    motorbit.Motors.M4, DigitalPin.P2,
    motorbit.Motors.M3, DigitalPin.P0,
    4.8, 4.8,
    8.8, 270
)
motorbit.setupArm(motorbit.Servos.S2,30,150,motorbit.Servos.S1,30,180)


// basic.showNumber(motorbit.getDegrees())
//motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 100)

// motorbit.MotorRun(motorbit.Motors.M3,50)
// basic.pause(10)
// motorbit.MotorRun(motorbit.Motors.M3, 20)
// basic.pause(2000)
// motorbit.MotorRun(motorbit.Motors.M3, 50)
// basic.pause(2000)

// motorbit.driveDistanceStraight(30, 10)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 20)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 30)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 40)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 50)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 60)
// basic.pause(500)
// motorbit.driveDistanceStraight(30, 70)
// basic.pause(500)
// motorbit.driveDistanceStraight(-40, 100)
// basic.pause(500)
// motorbit.rotateToDegrees(180)
// basic.pause(500)
// motorbit.headingToDegrees(90)
// motorbit.resetYaw()
input.onButtonPressed(Button.A, function () {
    motorbit.resetYaw()
    motorbit.Servo(motorbit.Servos.S1, 90)
    motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 150)
    motorbit.headingToDegrees(90, 120)
    motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 150)
    motorbit.rotateToDegrees(180, 120)
    motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 150)
    motorbit.turnRightForDegrees(90, 120)
    motorbit.closeGripperWithSpeed(5)
    motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 150)
    motorbit.openGripper()
    motorbit.Servo(motorbit.Servos.S1, 90)

})
input.onButtonPressed(Button.B, function () {
    // motorbit.headingToDegrees(180)
    // basic.showNumber(motorbit.getDegrees())
    // basic.pause(500)
    // motorbit.headingToDegrees(270)
    // basic.showNumber(motorbit.getDegrees())

    motorbit.resetYaw()
    motorbit.turnLeftForDegrees(90,120)

    // motorbit.headingToDegrees(0, 120)
    // basic.showNumber(motorbit.getDegrees())

    // motorbit.openGripper()
})


basic.forever(function () {
    

//     motorbit.MotorRun(motorbit.Motors.M1, 255)
//     motorbit.MotorRun(motorbit.Motors.M2, 255)
//     motorbit.MotorRun(motorbit.Motors.M3, 255)
//     motorbit.MotorRun(motorbit.Motors.M4, 255)
//     basic.pause(1000)
//     motorbit.MotorRun(motorbit.Motors.M1, -255)
//     motorbit.MotorRun(motorbit.Motors.M2, -255)
//     motorbit.MotorRun(motorbit.Motors.M3, -255)
//     motorbit.MotorRun(motorbit.Motors.M4, -255)
//     basic.pause(1000)
//     motorbit.MotorRun(motorbit.Motors.M1, 0)
//     motorbit.MotorRun(motorbit.Motors.M2, 0)
//     motorbit.MotorRun(motorbit.Motors.M3, 0)
//     motorbit.MotorRun(motorbit.Motors.M4, 0)
//     basic.pause(1000)
})
