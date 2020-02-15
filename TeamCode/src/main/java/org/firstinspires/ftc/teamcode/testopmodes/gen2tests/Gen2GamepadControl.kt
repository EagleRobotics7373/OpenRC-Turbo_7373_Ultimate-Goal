package org.firstinspires.ftc.teamcode.testopmodes.gen2tests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.legacyconfig.MisumiRobot

@TeleOp(group="Test")
class Gen2GamepadControl : OpMode() {
    lateinit var robot : MisumiRobot
    override fun init() {
        robot = MisumiRobot(hardwareMap)
    }

    override fun loop() {
        if (gamepad1.left_bumper) {
            robot.intakeLiftLeft.power = -gamepad1.left_stick_y.toDouble()
            robot.intakeLiftRight.power = gamepad1.left_stick_y.toDouble()
        } else if (gamepad1.right_bumper) {
            robot.intakeLiftLeft.power = -gamepad1.left_stick_y.toDouble()
            robot.intakeLiftRight.power = gamepad1.right_stick_y.toDouble()
        } else {
            robot.intakeLiftLeft.power = 0.0
            robot.intakeLiftRight.power = 0.0
        }

        robot.intakePivot.power = -gamepad2.left_stick_y.toDouble()
    }
}