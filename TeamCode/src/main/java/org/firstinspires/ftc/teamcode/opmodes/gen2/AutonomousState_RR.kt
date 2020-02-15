package org.firstinspires.ftc.teamcode.opmodes.gen1

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.Position.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer
import org.firstinspires.ftc.teamcode.opmodes.gen2.AutonomousConstants.*

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous State (Kotlin + RR)", group = "Main")
class AutonomousState_RR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtMisumiRobot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<PixelStatsPipeline>

    private lateinit var player          : ExtDirMusicPlayer

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
        robot = ExtMisumiRobot(hardwareMap)
        imuController = robot.imuControllerA
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                PixelStatsPipeline(PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


        robot.autoBlockIntakeRear.pivotIn18()
        robot.autoBlockIntakeRear.releaseBlock()

        robot.autoBlockIntakeFront.pivotIn18()
        robot.autoBlockIntakeFront.releaseBlock()

        robot.foundationGrabbersFront.unlock()


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
        player = ExtDirMusicPlayer(musicFile, true)
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
        cvContainer.camera.closeCameraDevice()
        player.stop()
    }

//    override fun internalPostLoop() {
//        super.internalPostLoop()
//        player.stop()
//    }

    fun doParkOnlyAuto() {
        sleep(delayBeforeParking * 1000.toLong())
        drive(0.0, 12.0, 0.5)
    }

    fun doCrossField() {
//        cvContainer.pipeline.tracking = true
//        while (cvContainer.pipeline.tracking);
//
//        val skystonePosition = cvContainer.pipeline.skystonePos

//        val skystonePositionFromWall =
//                when (allianceColor) {
//                    RED ->
//                        when (skystonePosition) {
//                            NULL  -> 0 // Not in camera frame
//                            LEFT  -> 1 // LEFT
//                            else  -> 2 // RIGHT, or unknown
//                        }
//                    BLUE ->
//                        when (skystonePosition) {
//                            RIGHT -> 0 // RIGHT
//                            LEFT  -> 1 // LEFT
//                            else  -> 2 // Not in camera frame, or unknown
//                        }
//                }

        val stonesOrder = arrayOf(1, 5, 4, 3).take(NUM_STONES)
//            when (skystonePositionFromWall) {
//                0    -> arrayOf(0, 3, 5)
//                1    -> arrayOf(1, 4, 5)
//                else -> arrayOf(2, 5, 4)
//            }

        val nextToStonePosY = RR_NEXT_TO_STONE_Y reverseIf RED
        val drivingAgainstBridgePosY = RR_AGAINST_BRIDGE_Y reverseIf RED
        val againstFoundationY = RR_AGAINST_FOUNDATION_Y reverseIf RED

        val stonePositionsFromWall = arrayOf(
                -62.0, -55.0, -48.0, -42.0, -31.0, -23.0)

        val foundationPlacementPositions = arrayOf(
                42.0, 46.0, 51.0, 52.0)

        robot.holonomicRR.poseEstimate =
                if (allianceColor == BLUE) Pose2d(-38.5, 63.0, 0.0)
                                      else Pose2d(-32.0, -63.0, 180.0.toRadians())

        builder()
                .strafeTo(Vector2d(stonePositionsFromWall[stonesOrder[0]], nextToStonePosY))
                .buildAndRun(Pair(0.3) { robot.autoBlockIntakeRear.pivotMid(); robot.autoBlockIntakeRear.grabberMid(); })

        for (i in stonesOrder.indices) {
            doBlockGrab(robot.autoBlockIntakeRear)
            builder()
                    .strafeTo(Vector2d(-16.0, drivingAgainstBridgePosY))
                    .strafeTo(Vector2d(18.0, drivingAgainstBridgePosY))
                    .strafeTo(Vector2d(foundationPlacementPositions[i], againstFoundationY))
                    .buildAndRun(Pair(0.7) { robot.autoBlockIntakeRear.pivotMid(); })

            doBlockRelease(robot.autoBlockIntakeRear)

            if (i < stonesOrder.size - 1) {
                builder()
                        .strafeTo(Vector2d(18.0, drivingAgainstBridgePosY))
                        .strafeTo(Vector2d(-16.0, drivingAgainstBridgePosY))
                        .strafeTo(Vector2d(stonePositionsFromWall[stonesOrder[i + 1]], nextToStonePosY))
                        .buildAndRun(Pair(0.7) { robot.autoBlockIntakeRear.pivotMid(); robot.autoBlockIntakeRear.grabberMid(); })

            } else {
                robot.autoBlockIntakeRear.releaseBlock()
                robot.autoBlockIntakeRear.pivotUp()
                if (doFoundationPull) {

                    builder().strafeTo(Vector2d(52.0, 40.0 reverseIf RED)).buildAndRun()
                    robot.foundationGrabbersFront.mid()
                    robot.holonomicRR.turnSync((-90.0).toRadians())
                    builder().strafeTo(Vector2d(52.0, 30.0 reverseIf RED)).buildAndRun()
                    robot.foundationGrabbersFront.lock()
                    sleep(350)
                    timeDrive(0.4, -0.6, -0.7, 1100)
                    timeDrive(0.0, 0.7, -0.15, 700)
                    robot.foundationGrabbersFront.unlock()

                    builder()
                            .strafeTo(Vector2d(0.0, 48.0 reverseIf RED))
                            .buildAndRun()

                } else {
                    builder()
                            .strafeTo(Vector2d(0.0, 48.0 reverseIf RED))
                            .buildAndRun()

                    break
                }
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
        sleep(600)
        grabber.grabBlock()
        sleep(700)
        grabber.pivotUp()
        sleep(400)
    }

    private fun doBlockRelease(grabber: AutoBlockIntake) {
        grabber.grabberMid()
        sleep(350)
        grabber.pivotUp()
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

    private fun builder() = robot.holonomicRR.trajectoryBuilder()

    private fun BaseTrajectoryBuilder.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}