package org.firstinspires.ftc.teamcode.testopmodes.sensortests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.MisumiRobot;
@Disabled
@TeleOp(name="Distance Sensor Test", group="Test")
public class DistanceSensorTests extends LinearOpMode {

    MisumiRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MisumiRobot(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addData("front (cm)", robot.distanceSensor_front.getDistance(DistanceUnit.CM));
//            telemetry.addData("rear  (cm)", robot.distanceSensor_rear.getDistance(DistanceUnit.CM));
//            telemetry.addData("side  (cm)", robot.distanceSensor_side.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
