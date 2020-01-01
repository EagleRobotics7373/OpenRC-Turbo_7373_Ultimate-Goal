package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstants {
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 0.5;
    public static double TRACK_WIDTH = 1;

    public static double MAX_RPM = 1150;

    public static double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static double kA = 0.0;
    public static double kStatic = 0.0;

    public static boolean RUN_USING_ENCODER = true;

    public static DriveConstraints BASE_CONSTRAINTS =
            new DriveConstraints(
                    30.0, 30.0, 0.0,
                    Math.PI, Math.PI, 0.0
            );

    public static PIDCoefficients TRANSLATIONAL_PID =
            new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients HEADING_PID =
            new PIDCoefficients(0, 0, 0);

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MAX_RPM * 0.85;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / 145.6;
    }




}
