package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@TeleOp(group="Test")
public class EncoderWatchOpMode extends OpMode {
    BasicRobot robot;
    MultipleTelemetry telem;
    @Override
    public void init() {
        robot = new BasicRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        telem.addData("flm cp", robot.frontLeftMotor.getCurrentPosition());
        telem.addData("frm cp", robot.frontRightMotor.getCurrentPosition());
        telem.addData("blm cp", robot.backLeftMotor.getCurrentPosition());
        telem.addData("brm cp", robot.backRightMotor.getCurrentPosition());
        telem.addData("odoX cp raw", robot.odometryXAxis.getDistanceRaw());
        telem.addData("oxoX cp in", robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH));
        telem.update();
    }
}
