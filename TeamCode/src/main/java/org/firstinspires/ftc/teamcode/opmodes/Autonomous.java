package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;
import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;
import org.firstinspires.ftc.teamcode.library.functions.Point3D;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionInitializer;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VuforiaController;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Main")
public class Autonomous extends LinearOpMode {
    BasicRobot robot;
    IMUController imuController;
    @Override
    public void runOpMode() throws InterruptedException {

        /*
                Initialize main autonomous variables
         */
        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);
        AutoMenuControllerIterative menuController = new AutoMenuControllerIterative(telemetry);
        robot.intakeBlockGrabber.release();

        /*
                Operate telemetry menu
         */
        while (!isStarted() && !isStopRequested()) {
           if (gamepad1.dpad_up) {
               menuController.menu.previousItem();
               while (gamepad1.dpad_up && !isStopRequested());
           } else if (gamepad1.dpad_down) {
               menuController.menu.nextItem();
               while (gamepad1.dpad_down && !isStopRequested());
           } else if (gamepad1.dpad_left) {
                menuController.menu.iterateBackward();
                while (gamepad1.dpad_left && !isStopRequested());
           } else if (gamepad1.dpad_right) {
               menuController.menu.iterateForward();
               while (gamepad1.dpad_right && !isStopRequested());
           }
        }

        waitForStart();
        double startLeftDist = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        if (!isStopRequested()) {
        /*
                RoboSpotify
         */
            ExtDirMusicPlayer player = new ExtDirMusicPlayer(menuController.getMusicFile(), true);
            player.play();

            PIDFCoefficients pidf = ((DcMotorEx)robot.frontLeftMotor).getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("p", pidf.p);
            telemetry.addData("i", pidf.i);
            telemetry.addData("d", pidf.d);
            telemetry.addData("f", pidf.f);
            telemetry.update();
//            sleep(5000);
        /*
                Robot Actions
         */


            if (menuController.getParkOnly()) {
                sleep(menuController.getDelayBeforeParking()*1000);
                if (!isStopRequested()) {
                    if (menuController.getStartingPosition()==FieldSide.WAFFLE_SIDE) {
                        if (menuController.getAllianceColor()==AllianceColor.RED)   drive(36, 0, 0.5);
                        else                                                        drive(-36, 0, 0.5);
                    }

                    if (menuController.getStartingPosition()==FieldSide.LOADING_ZONE) {
                        if (menuController.getAllianceColor()==AllianceColor.RED) drive(0, -20, 0.5);
                        else                                                      drive(0, 20, 0.5);
                    }
                }

            }
            else{
                if (menuController.getStartingPosition()== FieldSide.WAFFLE_SIDE) {
                    if (menuController.getAllianceColor()== AllianceColor.RED) {
                        if (menuController.getBuildingSiteSlide()) drive(-24, 0, 0.8);
                        // Drive forward to clear the wall
        //                drive(0, 5, 0.7);
        //                sleep(500);
        //                // Rotate 180 degrees using IMU PI controller
        //                imuPIRotate(180);

                        // Find distance away from the wall (REMOVED)
//                        double distWall = 42 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                        // Drive to the foundation
                        drive(15 - robot.leftDistanceSensor.getDistance(DistanceUnit.INCH), -29, 0.4);
                        sleep(250);

                        // Deploy the foundation grabber, grabbing the foundation
                        robot.foundationGrabbers.lock();
                        sleep(1000);

                        // Drive back to the wall
                        drive(0, 32, 0.7);
//                        timeDrive(0, 0.5, 0, 2000);
                        // Release the foundation grabbers
                        robot.foundationGrabbers.unlock();
                        sleep(500);


                        if (menuController.getParkAfterTask()) {
                            // Drive toward the alliance bridge to start moving around the foundation
                            drive(30, 0, 0.2);
                            // Drive parallel to the bridges to move to the other side of the foundation
                            drive(0, -18, 0.2);
                            drive(-10, 0, 0.2);

                            drive(60, 14, .7);

                            imuPIRotate(90);
                            telemetry.addData("heading", MathExtensionsKt.toDegrees(imuController.getHeading()));
                            telemetry.update();
                            sleep(2000);

//                            // Park under the bridge
//                            drive(23, 0, 0.6);
//                            if (menuController.getParkNearDS()) timeDrive(0,0.3, 0, 1000);
//                            else timeDrive(0, -0.3, 0, 500);
                        }

                    } else { // Blue side waffle
                        if (menuController.getBuildingSiteSlide()) drive(24, 0, 0.7);
                        // Drive forward to clear the wall
        //                drive(0, 5, 0.4);
        //                sleep(500);
                        // Find distance away from the wall
        //                double distWall = 44 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                        // Drive to the foundation
                        drive(0, -29, 0.4);
                        sleep(250);
                        telemetry.addData("blab blab blab", "wow taco");
                        telemetry.update();
                        // Deploy the foundation grabber, grabbing the foundation
                        robot.foundationGrabbers.lock();
                        sleep(2000);

                        // Drive back to the wall
//                        drive(0, 36, 0.2);
                        timeDrive(0, 0.5, 0, 2000);

                        // Release the foundation grabbers
                        robot.foundationGrabbers.unlock();
                        sleep(500);
                        if (menuController.getParkAfterTask()) {
                            // Drive toward the alliance bridge to start moving around the foundation
                            drive(-35, 0, 0.2);
                            // Drive parallel to the bridges to move to the other side of the foundation
                            drive(0, -17, 0.2);
                            //push foundation
                            drive(20, 0, 0.2);
                            sleep(1000);
                            // drive back
                            drive(-26, 0, 0.2);
                            if(menuController.getParkNearDS()) drive(0, 24,0.2);
                            else {
                                timeDrive(0, -0.4, 0, 500);
                                sleep(500);
                                robot.holonomic.stop();
                            }
                        }

                    }
                }
                else { // FIELD POSITION IS LOADING ZONE!!!



                    // Lift arm
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            doArmLift(1.68);
                        }
                    }).start();

                    drive(0, 28, .4);
                    drive(-25, 0, .4);
                    drive(-5, 0, .2);
                    drive(0, 5, .3);

                    Position skystonePosition = Position.RIGHT;
                    boolean leftIsStone  = ((double)(robot.leftColorSensor.red()+robot.leftColorSensor.green())/2 > 1.5*robot.leftColorSensor.blue());
                    boolean rightIsStone = ((double)(robot.rightColorSensor.red()+robot.rightColorSensor.green())/2 > 1.5*robot.rightColorSensor.blue());

                    if (leftIsStone & !rightIsStone)  {
                        skystonePosition = Position.CENTER;
                        drive(7,0,0.4);
                    } else if (!leftIsStone & rightIsStone) {
                        skystonePosition = Position.LEFT;
                        drive(22.5,3,0.4);
                    } else {
                        drive(7,0,0.4);
                    }

                    drive(0,5,0.6);
                    robot.intakeBlockGrabber.hold();
                    sleep(500);
                    robot.intakeBlockManipulator.setPower(1);
                    sleep(250);
                    drive(0, -9,.4);
//                    drive(95,0, 0.7);
//                    timeDrive(0.6, 0, 0, 500);
                    {
                        double p1 = 0.0175;
                        double i1 = 0.001;
                        double d1 = 0.0;
                        double c1 = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);
                        double t1 = 15.0;
                        double errorSum = 0;


                        double p2 = 1.0;
                        double i2 = 0.0;
                        double d2 = 0.0;
                        double t2 = imuController.getHeading();

                        while ((c1 = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH)) > t1 & opModeIsActive() & !Double.isNaN(c1)) {
                            double e1 = c1 - t1;
                            double e2 = imuController.getHeading() - t2;
                            if (e1 <= 15) {
                                errorSum += e1;
                            }
                            robot.holonomic.runWithoutEncoder(p1 * e1 + i1 * errorSum, 0, p2 * e2);
                            telemetry.addData("x target", t1);
                            telemetry.addData("x current", c1);
                            telemetry.addData("x error", e1);
                            telemetry.addData("heading target", t2);
                            telemetry.addData("heading error", e2);
                            telemetry.update();
                        }
                    }

                    robot.holonomic.stop();

                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            doArmLift(1.272);
                        }
                    }).start();

                    drive(0, 12, .5);
                    robot.intakeBlockGrabber.release();
                    sleep(1000);
                    drive(0, -7, 0.5);
                    double temp = robot.frontDistanceSensor.getDistance(DistanceUnit.INCH);
                    turn(180, 0.5);
                    robot.foundationGrabbers.setPosition(0.40);
                    sleep(500);
                    drive(0, -temp * .9333 + .4216, 0.5);
                    robot.foundationGrabbers.lock();
                    sleep(1000);
                    robot.intakePivotMotor.setPower(0);
//                    drive(0, 35, 0.8);
//                    drive(0, 5, .3);
                    timeDrive(0, 0.5, 0, 2000);

//                        timeDrive(0, 0.5, 0, 2000);
                    // Release the foundation grabbers
                    robot.foundationGrabbers.unlock();
                    sleep(1000);
                    // Drive toward the alliance bridge to start moving around the foundation
                    drive(33, 0, 0.2);
                    // Drive parallel to the bridges to move to the other side of the foundation
                    drive(0, -18, 0.2);
                    drive(-25, 0, 0.2);

                    drive(40, 14, .7);

                    imuPIRotate(90);
                    telemetry.addData("heading", MathExtensionsKt.toDegrees(imuController.getHeading()));
                    telemetry.update();
                    sleep(2000);
                }
            }
        /*
                End of OpMode - close resources
         */
            player.stop();
        }
    }

    public void imuPIRotate(double angle) {
        double currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        double Kp = .015; // Proportional Constant
        double Ki = .00; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<4) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);

            if (currentValue < 0) {
                et = -(Math.abs(targetValue) - Math.abs(currentValue));
            } else {
                et = targetValue - currentValue;
            }


            if (Kp * et > .8) {
                proportionPower = .8;
            } else {
                proportionPower = Kp * et;
            }

            if (Math.abs(et) < 45) {
                errorSum += et;
            }

            integralPower = Ki * errorSum;

            power = -(proportionPower + integralPower);
            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            robot.holonomic.runWithoutEncoder(0,0,power*0.30);
            telemetry.update();
        }
        robot.holonomic.stop();
    }

    private void doArmLift(double target) {
        double currentValue = 2.0;
        double targetValue = target;
        double PPower = 2.0;
        double originalRuntime = getRuntime();
        while ((currentValue=robot.intakePivotPotentiometer.getVoltage()) > targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<1) {
            robot.intakePivotMotor.setPower(-PPower * (targetValue-robot.intakePivotPotentiometer.getVoltage()));
        }
        robot.intakePivotMotor.setPower(0.07);
    }

    private void timeDrive(double x, double y, double z, long timeMs) {
        robot.holonomic.runWithoutEncoder(x, y, z);
        sleep(timeMs);
        robot.holonomic.stop();
    }

    private void turn(double degrees, double power) {
        robot.holonomic.turnUsingEncoder(180, 0.5);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime < 2);
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime<3) {

        }
    }


}