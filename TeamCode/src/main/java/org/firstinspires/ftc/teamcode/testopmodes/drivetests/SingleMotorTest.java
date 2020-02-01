package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SingleMotorTest extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "singlemotor");
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
    }
}
