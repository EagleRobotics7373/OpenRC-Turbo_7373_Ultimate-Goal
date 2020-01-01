package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@TeleOp(group="Test")
public class EncoderResetOpMode extends LinearOpMode {
    BasicRobot robot;
    MultipleTelemetry telem;
    @Override
    public void runOpMode() {
        robot = new BasicRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rearOdometry.resetHWCounter();
        robot.leftOdometry.resetHWCounter();
        robot.rearOdometry.resetHWCounter();

//        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
