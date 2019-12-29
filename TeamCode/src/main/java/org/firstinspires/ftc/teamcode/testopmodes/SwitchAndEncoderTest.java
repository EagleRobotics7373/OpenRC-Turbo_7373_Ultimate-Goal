package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@TeleOp(group="Test")
public class SwitchAndEncoderTest extends OpMode {
    DigitalChannel limitSwitch;
    DcMotor encoder;
    MultipleTelemetry telem = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    @Override
    public void init() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        encoder = hardwareMap.get(DcMotor.class, "encoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telem.addData("limit state", limitSwitch.getState());
        telem.addData("encoder pos", encoder.getCurrentPosition());
        telem.addData("encoder pos deg", ((double)encoder.getCurrentPosition()) / 8192 * 360);
        telem.update();
    }
}
