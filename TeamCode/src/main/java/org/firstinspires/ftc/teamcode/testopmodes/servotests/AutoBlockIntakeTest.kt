package org.firstinspires.ftc.teamcode.testopmodes.servotests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.legacyconfig.MisumiRobot

@TeleOp
class AutoBlockIntakeTest: LinearOpMode() {
    override fun runOpMode() {
        var robot = MisumiRobot(hardwareMap)
        waitForStart()
        while(opModeIsActive()) {
            when {
                gamepad1.a -> robot.autoBlockIntakeFront.pivotUp()
                gamepad1.b -> robot.autoBlockIntakeFront.pivotDown()
                gamepad1.x -> robot.autoBlockIntakeFront.grabBlock()
                gamepad1.x -> robot.autoBlockIntakeFront.releaseBlock()
            }
            when {
                gamepad2.a -> robot.autoBlockIntakeRear.pivotUp()
                gamepad2.b -> robot.autoBlockIntakeRear.pivotDown()
                gamepad2.x -> robot.autoBlockIntakeRear.grabBlock()
                gamepad2.x -> robot.autoBlockIntakeRear.releaseBlock()
            }
        }
    }
}