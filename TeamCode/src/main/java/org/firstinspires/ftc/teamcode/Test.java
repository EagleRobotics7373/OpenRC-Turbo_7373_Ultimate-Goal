package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.FunctionalExtensionsKt;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;

@Config
@TeleOp(name = "Test", group = "Sensor")
public class Test extends LinearOpMode {

    public static double PPower = 0;
    public static double constPower = .12;

    BasicRobot robot;
    IMUController imuController;

    @Override public void runOpMode() {

        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // wait for the start button to be pressed
        waitForStart();

//        while (opModeIsActive()) {
//            turn(180, 0.5);
//            System.out.println("BEN");
//            sleep(500);
//        }

        doArmLift();

        while (opModeIsActive()) {
            telemetry.addData("voltage", robot.intakePivotPotentiometer.getVoltage());
            telemetry.update();

        }
    }

    private void doArmLift() {
        double currentValue = robot.intakePivotPotentiometer.getVoltage();
        double targetValue = 1.3;
        double kP = PPower;
        double originalRuntime = getRuntime();
        while ((currentValue=robot.intakePivotPotentiometer.getVoltage()) > targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<1) {
            robot.intakePivotMotor.setPower(-kP * (targetValue-robot.intakePivotPotentiometer.getVoltage()));
        }
        robot.intakePivotMotor.setPower(constPower);
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime < 3) {

        }
    }

    private void turn(double degrees, double power) {
        robot.holonomic.turnUsingEncoder(180, 0.5);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime < 3);
    }

    public void imuPIRotate(double angle) {
        double currentValue = FunctionalExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        // Try Kp constant of .008
        double Kp = .03; // Proportional Constant
        double Ki = .00; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<4) {
            currentValue = FunctionalExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);


//            if (currentValue < 0) {
//                et = -(Math.abs(targetValue) - Math.abs(currentValue));
//            } else {
//                et = targetValue - currentValue;
//            }


//            if (Kp * et > .8) {
//                proportionPower = .8;
//            } else {
//                proportionPower = Kp * et;
//            }


            if (currentValue < 0) {
                currentValue += 360;
            }

            et = targetValue - currentValue;

            if (Math.abs(et) < 45) {
                errorSum += et;
            }

            proportionPower = Kp * et;
            integralPower = Ki * errorSum;
            power = -(proportionPower + integralPower);

            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            robot.holonomic.runWithoutEncoder(0,0,power);
            telemetry.update();
        }
        robot.holonomic.stop();
    }
}