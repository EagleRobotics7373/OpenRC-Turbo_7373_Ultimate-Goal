package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.STRAIGHT_TEST_DIST

@Autonomous(group="rr_cfg")
class StraightTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = OdometryRobot(hardwareMap)
        val drive = robot.holonomicRoadRunner

        waitForStart()

        val trajectory =
                drive.trajectoryBuilder
                        .forward(STRAIGHT_TEST_DIST)
                        .build()

        if (isStopRequested) return

        drive.followTrajectorySync(trajectory)
    }

}