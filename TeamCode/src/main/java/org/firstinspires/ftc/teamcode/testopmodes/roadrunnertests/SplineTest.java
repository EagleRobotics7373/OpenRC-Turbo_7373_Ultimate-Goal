package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.MisumiRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "rr_cfg")
@Config
public class SplineTest extends LinearOpMode {
    public static double distX = 10;
    public static double distY = 10;
    public static boolean reverse = false;
    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicRR drive = new MisumiRobot(hardwareMap).holonomicRR;

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(reverse)
                        .splineTo(new Pose2d(distX, distY, 0))
                        .build()
        );

//        sleep(2000);
//
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(0, 0, 0))
//                        .build()
//        );
    }
}
