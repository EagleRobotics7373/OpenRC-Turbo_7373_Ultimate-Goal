package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@TeleOp(group="Test")
public class EncoderResetOpMode extends LinearOpMode {
    BasicRobot robot;
    MultipleTelemetry telem;
    @Override
    public void runOpMode() {
        robot = new BasicRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odometryXAxis.resetHWCounter();
//        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
