package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.STRAIGHT_TEST_DIST

@TeleOp(group="rr_cfg")
class StraightTestWithMidAction : LinearOpMode() {
    override fun runOpMode() {
        val robot = ExtMisumiRobot(hardwareMap)
        val drive = robot.holonomicRR
        robot.autoBlockIntakeRear.pivotUp()
        waitForStart()

        if (isStopRequested) return

        val trajectory =
                drive.trajectoryBuilder()
                        .forward(STRAIGHT_TEST_DIST)
                        .build()

        drive.followTrajectorySync(trajectory, listOf(Pair(0.5, {robot.autoBlockIntakeRear.pivotMid()})))
        while (!isStopRequested) drive.update()
    }

}