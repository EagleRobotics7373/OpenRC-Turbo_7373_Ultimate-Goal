package org.firstinspires.ftc.teamcode.testopmodes.servotests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@TeleOp(name = "Servo Test", group = "Test")
public class AutoBlockIntakeTest extends LinearOpMode{

    BasicRobot robot;

    @Override
    public void runOpMode(){
        robot = new BasicRobot(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.x) robot.autoBlockIntake.grabBlock();
            else if (gamepad2.y) robot.autoBlockIntake.releaseBlock();
            else if (gamepad2.dpad_down) robot.autoBlockIntake.pivotDown();
            else if (gamepad2.dpad_up) robot.autoBlockIntake.pivotUp();
            else if (gamepad2.dpad_right) robot.autoBlockIntake.pivotMid();
        }

    }
}