package org.firstinspires.ftc.teamcode.testopmodes.servotests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Complex Servo Test", group="Test")
public class ComplexServoTestOpMode extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.left_trigger);
        telemetry.addData("power", gamepad1.left_trigger);
    }
}
