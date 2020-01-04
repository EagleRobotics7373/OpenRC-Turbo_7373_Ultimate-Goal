package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot;

@Config
@TeleOp
public class PositionalDriveTest extends LinearOpMode {
    public static double xDist = 0;
    public static double yDist = -10;
    public static double headingDegrees = 0;

    public static PIDCoefficients xCoeffs       = new PIDCoefficients(0.03, 1.5, 0);
    public static PIDCoefficients yCoeffs       = new PIDCoefficients(0.3, 0.01, 0);
    public static PIDCoefficients headingCoeffs = new PIDCoefficients(1.5, 0, 0);

    public static double xMax = 0.2;
    public static double yMax = 0.2;
    public static double headingMax = 0.2;

    public static double xRestrict = 5;
    public static double yRestrict = 5;
    public static double headingRestrict = Math.PI / 2;

    public static boolean servosLocked = false;

    public OdometryRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OdometryRobot(hardwareMap);

        waitForStart();

        if (servosLocked) robot.foundationGrabbers.lock();
        else robot.foundationGrabbers.unlock();

        robot.positionalHolonomic.runPositionSync(
                robot.positionalHolonomic.positionConstructor()
                        .vectorRelative(xDist, yDist, headingDegrees)
                        .setXAxisPID(xCoeffs)
                        .setYAxisPID(yCoeffs)
                        .setHeadingPID(headingCoeffs)
                        .setMaxPowers(xMax, yMax, headingMax)
                        .setSumRestrict(xRestrict, yRestrict, headingRestrict)
                        .build()
        );
    }
}
