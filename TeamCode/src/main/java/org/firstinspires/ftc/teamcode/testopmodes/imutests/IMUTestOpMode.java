package org.firstinspires.ftc.teamcode.testopmodes.imutests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;

@Config
@TeleOp (name= "IMU Test OpMode", group="Test")
public class IMUTestOpMode extends OpMode {
    IMUController imuControllerA;
    IMUController imuControllerB;

    public static AxesReference orientation = AxesReference.INTRINSIC;

    @Override
    public void init() {
        imuControllerA = new IMUController(hardwareMap, AxesOrder.ZYX);
        imuControllerB = new IMUController(hardwareMap, AxesOrder.ZYX, "imuB");
    }

    @Override
    public void loop() {
        Orientation aoA = imuControllerA.imuA.getAngularOrientation(orientation, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation aoB = imuControllerB.imuA.getAngularOrientation(orientation, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("A_first", aoA.firstAngle);
        telemetry.addData("A_second", aoA.secondAngle);
        telemetry.addData("A_third", aoA.thirdAngle);
        telemetry.addData("B_first", aoB.firstAngle);
        telemetry.addData("B_second", aoB.secondAngle);
        telemetry.addData("B_third", aoB.thirdAngle);
        telemetry.update();

    }
}
