package org.firstinspires.ftc.teamcode.opmodes.gen1

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.robotcore.MisumiRobot
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer
import org.firstinspires.ftc.teamcode.opmodes.gen2.AutonomousConstants.*

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous LT (Kotlin + RR)", group = "Main")
class AutonomousKotlinLT_RR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : MisumiRobot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<PixelStatsPipeline>

    /*
        VARIABLES: Menu Options
     */
    var allianceColor                       = _allianceColor
    var startingPosition                    = _startingPosition
    var musicFile                           = _musicFile
    var parkOnly                            = _parkOnly
    var visionDetector                      = PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV
    var delayBeforeParking                  = _delayBeforeParking
    var foundationSwivel                    = _foundationSwivel
    var doFoundationPull                    = _doFoundationPull
    var doParkAtEnd                                 = _doParkAtEnd
    var liftLowerOnly = false
    var doLiftLower = false
    var parkNearDS                          = false
    var parkAfterTask                       = true
    var foundationRedundancy                = true
    var pushAlliancePartner                 = true

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = MisumiRobot(hardwareMap)
        imuController = robot.imuControllerA
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                PixelStatsPipeline(PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


        robot.autoBlockIntakeRear.pivotUp()
        robot.autoBlockIntakeRear.grabBlock()
        robot.foundationGrabbersSide.unlock()
//        robot.foundationGrabbersFront.unlock()


        /*
            Operate reflective telemetry menu, releasing when OpMode starts or stops
         */
        operateMenu()


        /*
            OpMode has proceeded past init. If stop is requested, return runOpMode() early.
         */
        if (isStopRequested) return


        /*
            Prepare music and setup additional components.
         */
        val player = ExtDirMusicPlayer(musicFile, true)
        player.play()

        elapsedTime = ElapsedTime()

        cvContainer.pipeline.detector = visionDetector


        /*
            Perform actions
         */
        if (parkOnly) doParkOnlyAuto()
        else if (liftLowerOnly) doIntakeDrop()
        else doCrossField()


        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

        cvContainer.stop()
        player.stop()
    }

    fun doParkOnlyAuto() {
        sleep(delayBeforeParking * 1000.toLong())
        drive(0.0, 12.0, 0.5)
    }

    fun doCrossField() {
        cvContainer.pipeline.tracking = true
        while (cvContainer.pipeline.tracking);

        val skystonePosition = cvContainer.pipeline.skystonePos
        val modifiedSkystonePosition =
                when (skystonePosition) {
                    null -> Position.RIGHT
                    Position.CENTER -> Position.RIGHT
                    Position.LEFT -> Position.CENTER
                    Position.RIGHT -> Position.RIGHT
                    Position.NULL -> Position.LEFT
                }

//        val stonePositionsFromWall = listOf()

        robot.autoBlockIntakeRear.pivotUp()
        robot.autoBlockIntakeFront.pivotUp()

        val closeStoneXPositionsBlue = mapOf(
                Pair(Position.LEFT, 3.0+16),
                Pair(Position.CENTER, 3.0+8),
                Pair(Position.RIGHT, 3.0)
        )
        val wallStoneXPositionsBlue = closeStoneXPositionsBlue.mapValues { it.value - 24 }


        val closeStoneXPositionsRed = mapOf(
                Pair(Position.LEFT, 3.0),
                Pair(Position.CENTER, 3.0 - 8),
                Pair(Position.RIGHT, 3.0 - 16)
        )
        val wallStoneXPositionsRed = closeStoneXPositionsRed.mapValues { it.value + 24 }

        val atStoneY = -27.0
        val awayFromBridgeY = -20.5

        val foundationXPositions = listOf(80.0, 89.0)
        val foundationYPosition = -30.5

//        robot.holonomicRR.poseEstimate = Pose2d(-39.0, 60.0 reverseIf RED, 0.0)

        if (allianceColor == BLUE) {
            robot.autoBlockIntakeFront.grabBlock()
            robot.autoBlockIntakeRear.releaseBlock()

            if (doLiftLower) doIntakeDrop()

            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d((wallStoneXPositionsBlue[modifiedSkystonePosition]?:0.0) reverseIf RED, atStoneY))
                            .build()
            )
            doBlockGrab(robot.autoBlockIntakeRear)
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(foundationXPositions[0], foundationYPosition))
                            .build()
            )
            doBlockRelease(robot.autoBlockIntakeRear)
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(closeStoneXPositionsBlue[modifiedSkystonePosition]?:0.0, atStoneY))
                            .build()
            )
            robot.autoBlockIntakeRear.releaseBlock()
            sleep(450)
            doBlockGrab(robot.autoBlockIntakeRear)
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(foundationXPositions[1], foundationYPosition-0.5))
                            .build()
            )
            robot.foundationGrabbersSide.lock()
            sleep(700)
            doBlockRelease(robot.autoBlockIntakeRear)
            sleep(500)
            if (doFoundationPull) {
    //            robot.holonomicRR.followTrajectorySync(robot.holonomicRR.trajectoryBuilder()
    //                    .strafeTo(Vector2d(foundationXPositions[1], 0.0))
    //                    .build()
    //            )
                timeDrive(-1.0, 0.0, -0.45, 2500)
                timeDrive(1.0, 0.0, 0.0, 1000)
                robot.foundationGrabbersSide.unlock()
            }
            if (doParkAtEnd) {
                robot.holonomicRR.followTrajectorySync(
                        robot.holonomicRR.trajectoryBuilder()
                                .reverse()
                                .strafeTo(Vector2d(44.0, -23.5))
                                .build()
                )
            }
        } else {

            robot.autoBlockIntakeFront.grabberMid()
            robot.autoBlockIntakeRear.grabBlock()
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d((wallStoneXPositionsRed[modifiedSkystonePosition]?:0.0), atStoneY))
                            .build()
            )
            doBlockGrab(robot.autoBlockIntakeFront)

            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(-4.0, awayFromBridgeY))
                            .build()
            )

            if (doLiftLower) doIntakeDrop()

            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(foundationXPositions[1] reverseIf RED, foundationYPosition))
                            .build()
            )
            doBlockRelease(robot.autoBlockIntakeFront)
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(closeStoneXPositionsRed[modifiedSkystonePosition]?:0.0, atStoneY))
                            .build()
            )
            robot.autoBlockIntakeFront.releaseBlock()
            sleep(450)
            doBlockGrab(robot.autoBlockIntakeFront)
            robot.holonomicRR.followTrajectorySync(
                    robot.holonomicRR.trajectoryBuilder()
                            .strafeTo(Vector2d(28.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(57.0 reverseIf RED, awayFromBridgeY))
                            .strafeTo(Vector2d(foundationXPositions[0] reverseIf RED, foundationYPosition-1.0))
                            .build()
            )
            robot.foundationGrabbersSide.lock()
            sleep(700)
            doBlockRelease(robot.autoBlockIntakeFront)
            sleep(500)
            if (doFoundationPull) {
                //            robot.holonomicRR.followTrajectorySync(robot.holonomicRR.trajectoryBuilder()
                //                    .strafeTo(Vector2d(foundationXPositions[1], 0.0))
                //                    .build()
                //            )
                timeDrive(-1.0, 0.35, 0.3, 2000)
                robot.foundationGrabbersSide.unlock()
                sleep(500)
                timeDrive(1.0, 0.0, 0.0, 1000)
            }
            if (doParkAtEnd) {
                robot.holonomicRR.followTrajectorySync(
                        robot.holonomicRR.trajectoryBuilder()
                                .strafeTo(Vector2d(40.0 reverseIf RED, -3.0))
                                .build()
                )
            }
        }

    }

    /**
     * Creates and operates [ReflectiveTelemetryMenu] before the init period.
     * Controls code until [isStopRequested] or [isStarted] is true.
     */

    fun operateMenu() {
        val menu = ReflectiveTelemetryMenu(telem,
                ReflectiveMenuItemEnum("Alliance Color", ::allianceColor, AllianceColor.RED, AllianceColor.BLUE),
                ReflectiveMenuItemEnum("Starting Position", ::startingPosition, FieldSide.WAFFLE_SIDE, FieldSide.LOADING_ZONE),
                ReflectiveMenuItemEnum("Music", ::musicFile, *ExtMusicFile.values()),
                ReflectiveMenuItemBoolean("Foundation Swivel", ::foundationSwivel),
                ReflectiveMenuItemBoolean("Foundation Pull", ::doFoundationPull),
                ReflectiveMenuItemBoolean("Park at End", ::doParkAtEnd),
                ReflectiveMenuItemBoolean("Lift Lower Only", ::liftLowerOnly)
        )

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.call()    -> menu.previousItem()
                dpadDownWatch.call()  -> menu.nextItem()
                dpadLeftWatch.call()  -> menu.iterateBackward()
                dpadRightWatch.call() -> menu.iterateForward()
            }
            if (gamepad1.x && !doLiftLower) {
                robot.intakeLiftRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                doLiftLower = true
                robot.intakeLiftRight.targetPosition = -1195
                robot.intakeLiftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
                robot.intakeLiftRight.power = 0.3
                while (gamepad1.x && !isStarted);
                robot.intakeLiftRight.power = 0.0
                robot.intakeLiftRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                robot.intakeLiftRight.power = 0.0
            }
        }


    }

    private fun doBlockGrab(grabber: AutoBlockIntake) {
        grabber.pivotDown()
        sleep(550)
        grabber.grabBlock()
        sleep(650)
        if (grabber == robot.autoBlockIntakeFront) grabber.pivotUp()
        else grabber.pivotMid()
        sleep(700)
    }

    private fun doBlockRelease(grabber: AutoBlockIntake) {
//        grabber.pivotMid()
//        sleep(400)
        grabber.grabberMid()
        sleep(600)
        grabber.pivotMid()
    }

    /**
     * Robot strafes for desired distance until motors have reached target.
     * Forwards distances and power to [robot.holonomic]
     */
    private fun drive(x: Double, y: Double, power: Double) {
        robot.holonomic.runUsingEncoder(x, y, power)
        val originalRuntime = runtime
        @Suppress("StatementWithEmptyBody") while (opModeIsActive() && robot.holonomic.motorsAreBusy() && runtime - originalRuntime < 3) robot.holonomicRR.update();
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    /**
     * Robot strafes with specified power until desired time length is reached.
     * Forwards power to [robot.holonomic]
     */
    private fun timeDrive(x: Double, y: Double, z: Double, timeMs: Long) {
        val startingRuntime = System.currentTimeMillis()
        robot.holonomic.runWithoutEncoder(x, y, z)
        while (System.currentTimeMillis() - startingRuntime < timeMs) robot.holonomicRR.update()
        robot.holonomic.stop()
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)

    }

    private fun doIntakeDrop() {
        while (opModeIsActive()) {
            val intakeLiftCurrent = robot.intakeLiftRight.currentPosition
            val inputSpoolPower =
                        if (intakeLiftCurrent > 0.0) 0.0
                        else if (intakeLiftCurrent + 75 > 0.0) 0.20
                        else if (intakeLiftCurrent + 500 > 0.0) 0.45
                        else if (intakeLiftCurrent + 800 > 0.0) 0.45
                        else if (intakeLiftCurrent + 1200 > 0.0) 0.60
                        else if (intakeLiftCurrent + 2000 > 0.0) 0.75
                        else 1.0

            robot.intakeLiftLeft.power = -inputSpoolPower
            robot.intakeLiftRight.power = inputSpoolPower

            if (inputSpoolPower == 0.0) break
        }
    }

    /**
     * Reverses input number if [testColor] matches [allianceColor]
     */
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double {
        return this * if (allianceColor==testColor) -1.0 else 1.0
    }

    fun imuPIRotate(angle: Double) {
        var currentValue = imuController.getHeading().toDegrees()
        val targetValue = currentValue + angle

        val Kp = .02 // Proportional Constant
        val Ki = .0007 // Integral Constant
        var et: Double // Error
        var proportionPower: Double
        var integralPower: Double
        var power: Double
        var errorSum = 0.0
        val originalRuntime = runtime
        while (currentValue != targetValue && opModeIsActive() && runtime - originalRuntime < 4) {
            currentValue = imuController.getHeading().toDegrees()
            telemetry.addData("Current value", currentValue)
            telemetry.addData("Target value", targetValue)

            if (currentValue < 0) {
                et = -(Math.abs(targetValue) - Math.abs(currentValue))
            } else {
                et = targetValue - currentValue
            }


            if (Kp * et > .8) {
                proportionPower = .8
            } else {
                proportionPower = Kp * et
            }

            if (Math.abs(et) < 45) {
                errorSum += et
            }

            integralPower = Ki * errorSum

            power = -(proportionPower + integralPower)
            telemetry.addData("et", et)
            telemetry.addData("propPw", proportionPower)
            telemetry.addData("intPw", integralPower)
            telemetry.addData("errorsum", errorSum)
            telemetry.addData("Power", power)
            robot.holonomic.runWithoutEncoder(0.0, 0.0, power * 0.30)
            telemetry.update()
        }
        robot.holonomic.stop()
    }

}