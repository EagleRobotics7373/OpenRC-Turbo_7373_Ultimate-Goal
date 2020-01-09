package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.STRAIGHT_TEST_DIST

@Config
@TeleOp(group="rr_cfg")
class MultiDistanceTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = OdometryRobot(hardwareMap)
        val drive = robot.holonomicRoadRunner

        waitForStart()

        val trajectory =
                drive.trajectoryBuilder
                        .splineTo(Pose2d(70.0, -8.0))
                        .build()

        if (isStopRequested) return
for (i in 1..4) {
    drive.followTrajectorySync(drive.trajectoryBuilder
            .splineTo(Pose2d(70.0, -8.0))
            .build())
    sleep(0)
    drive.followTrajectorySync(drive.trajectoryBuilder
            .reverse()
            .splineTo(Pose2d(0.0, 0.0))
            .build())
    sleep(0)
}
        while (!isStopRequested) drive.update()
    }

}