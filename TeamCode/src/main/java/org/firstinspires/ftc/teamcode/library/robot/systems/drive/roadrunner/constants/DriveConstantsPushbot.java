package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstantsPushbot {
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 0.5;
    public static double TRACK_WIDTH = 14.0;

    public static double MAX_RPM = 1150;
    public static double TICKS_PER_REV = 145.6;

    public static double kV = /*0.0035 .00772*/  /* 1.0 / rpmToVelocity(getMaxRpm())*/ 0.006;
    public static double kA = 0.0;
    public static double kStatic = .014;

    public static Pose2d globalPoseEstimate = null;

    public static boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(30, 0, 10);

    public static DriveConstraints BASE_CONSTRAINTS =
            new DriveConstraints(
//                    50.0, 30.0, 40.0,
                    10.0, 10.0, 40.0,
                    Math.PI/2, Math.PI, 0.0
            );

    public static PIDCoefficients TRANSLATIONAL_X_PID =
            new PIDCoefficients(4.2, 0.0, 0.4);

    public static PIDCoefficients TRANSLATIONAL_Y_PID =
            new PIDCoefficients(3.5, 0.0, 0.1);

    public static PIDCoefficients HEADING_PID =
            new PIDCoefficients(4, 0.1, 0.05);

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}