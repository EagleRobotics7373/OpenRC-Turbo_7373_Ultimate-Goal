package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
import org.firstinspires.ftc.teamcode.opmodes.control.IMUPIDStrafer;

import kotlin.jvm.functions.Function0;

@Autonomous(name="Odometry Strafe Test", group="Test")
public class OdometryStrafeTest extends LinearOpMode {

    BasicRobot robot;
    IMUController imuController;
    MultipleTelemetry telem;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap);
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        robot.odometryXAxis.resetSWCounter();
        IMUPIDStrafer strafer = new IMUPIDStrafer(
                robot.holonomic,
                imuController,
                OdometryStrafeTestConfig.strafePIDCoefficients,
                OdometryStrafeTestConfig.angularPIDCoefficients,
                new Function0<Double>() {
                    @Override
                    public Double invoke() { return OdometryStrafeTestConfig.distance - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH); }
                }


        );
        double error;
        while ((error = strafer.getStrafeErrorFun().invoke()) > OdometryStrafeTestConfig.tolerance && opModeIsActive()) {
            telem.addData("error", error);
            telem.addData("errorSum", strafer.getStrafeErrorSum());
//            telem.addData("current pos norm", robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH));
//            telem.addData("current pos raw", robot.odometryXAxis.getDistanceRaw());
            telem.update();
            strafer.run();
        }

        robot.holonomic.stop();
    }
}

@Config
class OdometryStrafeTestConfig {
    public static double distance = 24;
    public static double tolerance = 0.3;
    public static PIDCoefficients strafePIDCoefficients = new PIDCoefficients(0.04, 0.0000013, 0.0);
    public static PIDCoefficients angularPIDCoefficients = new PIDCoefficients(1.0, 0.0, 0.0);
}
