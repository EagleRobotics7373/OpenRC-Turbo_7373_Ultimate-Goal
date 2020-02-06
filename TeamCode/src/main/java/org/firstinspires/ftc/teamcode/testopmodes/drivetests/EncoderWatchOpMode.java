package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.MisumiRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule;

@TeleOp(group="Test")
public class EncoderWatchOpMode extends OpMode {
    MisumiRobot robot;
    MultipleTelemetry telem;
    @Override
    public void init() {
        robot = new MisumiRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        telem.addData("flm cp", robot.frontLeftMotor.getCurrentPosition());
        telem.addData("frm cp", robot.frontRightMotor.getCurrentPosition());
        telem.addData("blm cp", robot.backLeftMotor.getCurrentPosition());
        telem.addData("brm cp", robot.backRightMotor.getCurrentPosition());

        addOdometryData(robot.odometryModuleRear, "Rear");
        addOdometryData(robot.odometryModuleLeft, "Left");

        telem.update();

    }

    private void addOdometryData(OdometryModule module, String name) {
        telem.addData("odo"+name+" cp raw", module.getDistanceRaw());
        telem.addData("oxo"+name+" cp in", module.getDistanceNormalized(DistanceUnit.INCH));
    }

}
