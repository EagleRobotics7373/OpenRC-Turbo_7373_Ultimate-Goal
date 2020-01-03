package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot;

@Config
@TeleOp
public class PositionalDriveTest extends LinearOpMode {
    public static double xDist = 10;
    public static double yDist = 10;

    public static PIDCoefficients xCoeffs       = new PIDCoefficients(1, 0, 0);
    public static PIDCoefficients yCoeffs       = new PIDCoefficients(1, 0, 0);
    public static PIDCoefficients headingCoeffs = new PIDCoefficients(1, 0, 0);


    public OdometryRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OdometryRobot(hardwareMap);

        waitForStart();

        robot.positionalHolonomic.runPositionSync(
                robot.positionalHolonomic.positionConstructor()
                .vectorRelative(xDist, yDist)
                        .setXAxisPID(xCoeffs)
                        .setYAxisPID(yCoeffs)
                        .setHeadingPID(headingCoeffs)
                .build()
        );
    }
}
