package org.firstinspires.ftc.teamcode.testopmodes.gen2tests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import java.lang.Thread.sleep

@TeleOp(group="Test")
class AutoBlockGrabberTest : OpMode() {
    lateinit var robot : ExtMisumiRobot
    lateinit var grabbers: List<AutoBlockIntake>

    override fun init() {
        robot = ExtMisumiRobot(hardwareMap)
        grabbers = listOf(robot.autoBlockIntakeFront, robot.autoBlockIntakeRear)
    }

    override fun loop() {
        when {
            // block grabbing
            gamepad1.x -> grabbers.forEach { it.grabBlock() }
            gamepad1.a -> grabbers.forEach { it.grabberMid() }
            gamepad1.y -> grabbers.forEach { it.releaseBlock() }

            // pivot arm
            gamepad2.x -> grabbers.forEach { it.pivotDown() }
            gamepad2.a -> grabbers.forEach { it.pivotMid() }
            gamepad2.y -> grabbers.forEach { it.pivotUp() }
            gamepad2.b -> grabbers.forEach { it.pivotIn18() }

            // full action
            gamepad1.dpad_down -> doBlockGrab(robot.autoBlockIntakeRear)
            gamepad1.dpad_up   -> doBlockRelease(robot.autoBlockIntakeRear)
            gamepad2.dpad_down -> doBlockGrab(robot.autoBlockIntakeFront)
            gamepad2.dpad_up   -> doBlockRelease(robot.autoBlockIntakeFront)
        }
    }

    private fun doBlockGrab(grabber: AutoBlockIntake) {
        grabber.pivotDown()
        sleep(400)
        grabber.grabBlock()
        sleep(400)
        grabber.pivotUp()
        sleep(400)
    }

    private fun doBlockRelease(grabber: AutoBlockIntake) {
        grabber.grabberMid()
        sleep(350)
        grabber.pivotUp()
    }
}