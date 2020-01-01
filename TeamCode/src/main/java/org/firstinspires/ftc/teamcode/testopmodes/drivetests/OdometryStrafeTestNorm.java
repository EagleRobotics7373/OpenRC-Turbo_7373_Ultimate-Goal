package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;

@Autonomous(name="Odometry Strafe Test Norm", group="Test")
public class OdometryStrafeTestNorm extends LinearOpMode {

    BasicRobot robot;
    IMUController imuController;
    MultipleTelemetry telem;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap);
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        double lastRuntime = 0.0;

        robot.rearOdometry.resetSWCounter();
        {
                        double p1 = OdometryStrafeTestNormConfig.strafeP;
                        double i1 = OdometryStrafeTestNormConfig.strafeI;
                        double c1 = robot.rearOdometry.getDistanceNormalized(DistanceUnit.INCH);
                        double t1 = robot.rearOdometry.getDistanceNormalized(DistanceUnit.INCH) + OdometryStrafeTestNormConfig.distance;
                        double errorSum = 0;


                        double p2 = OdometryStrafeTestNormConfig.angularP;
                        double i2 = OdometryStrafeTestNormConfig.angularI;
                        double t2 = imuController.getHeading();

                        double startingRuntime = getRuntime();
                        double e1;


                        do {
                            c1 = robot.rearOdometry.getDistanceNormalized(DistanceUnit.INCH);

                            e1 = t1-c1;

                            System.out.println("\n---------E1 = "+e1);
                            System.out.println("---------------C1 = "+c1);
                            System.out.println("starting runtime: "+startingRuntime + "    current: "+getRuntime()+ "   error: " + (getRuntime()-startingRuntime));
                            double e2 = imuController.getHeading() - t2;

                            errorSum += e1/(getRuntime()-lastRuntime);
                            lastRuntime = getRuntime();

                            robot.holonomic.runWithoutEncoder((p1 * e1 + i1 * errorSum), 0, p2 * e2);


                            telem.addData("x target", t1);
                            telem.addData("x current", c1);
                            telem.addData("x error", e1);
                            telem.addData("heading target", t2);
                            telem.addData("heading error", e2);
                            telem.update();
                        } while (c1 < t1  & opModeIsActive());

                        System.out.println("\n---------E1 = "+e1);
                        System.out.println("---------------C1 = "+c1);
                        System.out.println("starting runtime: "+startingRuntime + "    current: "+getRuntime()+ "   error: " + (getRuntime()-startingRuntime));
                        System.out.println("---------------FINAL C1"+c1);

                    }
        robot.holonomic.stop();
    }
}

@Config
class OdometryStrafeTestNormConfig {
    public static double distance = 20;
    public static double tolerance = 2;
    public static double strafeP = 1;
    public static double strafeI = 0;
    public static double angularP = 1;
    public static double angularI = 0;

}
