package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline
import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional.PositionConstructor.DriveProfile.*
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer
import org.firstinspires.ftc.teamcode.opmodes.AutonomousConstants.*

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous LT (Kotlin + RR)", group = "Main")
class AutonomousKotlinLT_RR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : OdometryRobot
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
                     var parkNearDS                          = false
                     var parkAfterTask                       = true
                     var foundationRedundancy                = true
                     var pushAlliancePartner                 = true

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = OdometryRobot(hardwareMap)
        imuController = robot.imuController
//        cvContainer = VisionFactory.createOpenCv(
//                VisionFactory.CameraType.WEBCAM,
//                hardwareMap,
//                PixelStatsPipeline(PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


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

//        cvContainer.pipeline.detector = visionDetector


        /*
            Perform actions
         */
        if (parkOnly) doParkOnlyAuto()
        else when (startingPosition) {
            FieldSide.WAFFLE_SIDE  -> if (foundationSwivel) doWaffleSwivelAuto() else doWaffleSideAuto()
            FieldSide.LOADING_ZONE -> doLoadingZoneAuto()
        }


        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRoadRunner.update()

//        cvContainer.stop()
        player.stop()
    }

    fun doParkOnlyAuto() {
        sleep(delayBeforeParking * 1000.toLong())
        drive(0.0, 12.0, 0.5)
    }

    fun doWaffleSideAuto() {
        robot.positionalHolonomic.runPositionSync(
                positionalConstructor()
                        .setDriveProfile(STRAFE_X)
                        .setSumRestrict(x = 5.5, y = 2.0)
                        .setXAxisPID(PIDCoefficients(0.012, 0.15, 0.0))
                        .setYAxisPID(PIDCoefficients(0.03, 0.17, 0.0))
                        .vectorRelative(30.0, 13.0)
                        .build()
        )

        robot.foundationGrabbers.lock()
        sleep(2000)

        robot.positionalHolonomic.runPositionSync(
                positionalConstructor()
                        .setDriveProfile(STRAFE_X)
                        .setSumRestrict(x = 5.0, y = 5.0)
                        .setXAxisPID(PIDCoefficients(0.01, 0.1, 0.0))
                        .setYAxisPID(PIDCoefficients(0.00, 0.0, 0.0))
                        .setMaxPowers(0.4, 0.0, 0.2)
                        .vectorRelative(-40.0, 0.0, 90.0)
                        .setIgnoreAxes(x = true, y = true, heading = false)
                        .build()
        )
    }

    fun doWaffleSwivelAuto() {

    }

    fun doLoadingZoneAuto() {
        robot.autoBlockIntake.releaseBlock()
        robot.autoBlockIntake.pivotMid()

        robot.holonomicRoadRunner.followTrajectorySync(
                robot.holonomicRoadRunner.trajectoryBuilder()
                        .strafeTo(Vector2d(12.0, -28.0))
                        .build()
        )
        robot.autoBlockIntake.pivotDown()
        sleep(550)
        robot.autoBlockIntake.grabBlock()
        sleep(550)
        robot.autoBlockIntake.pivotUp()
        sleep(700)
        robot.holonomicRoadRunner.followTrajectorySync(
                robot.holonomicRoadRunner.trajectoryBuilder()
                        .strafeTo(Vector2d(42.0, -22.0))
                        .strafeTo(Vector2d(96.0, -30.0))
                        .build()
        )

        robot.autoBlockIntake.pivotDown()
        sleep(400)
        robot.autoBlockIntake.releaseBlock()
        sleep(800)

        robot.autoBlockIntake.pivotUp()
        robot.holonomicRoadRunner.followTrajectorySync(
                robot.holonomicRoadRunner.trajectoryBuilder()
                        .strafeTo(Vector2d(42.0, -22.0))
                        .strafeTo(Vector2d(-12.0, -28.75))
                        .build()
        )

        robot.autoBlockIntake.pivotDown()
        sleep(550)
        robot.autoBlockIntake.grabBlock()
        sleep(550)
        robot.autoBlockIntake.pivotUp()
        sleep(700)
        robot.holonomicRoadRunner.followTrajectorySync(
                robot.holonomicRoadRunner.trajectoryBuilder()
                        .strafeTo(Vector2d(42.0, -22.0))
                        .strafeTo(Vector2d(87.0, -31.5))
                        .build()
        )

        robot.autoBlockIntake.pivotDown()
        sleep(400)
        robot.autoBlockIntake.releaseBlock()
        sleep(600)
        robot.autoBlockIntake.pivotUp()
        robot.holonomicRoadRunner.followTrajectorySync(
                robot.holonomicRoadRunner.trajectoryBuilder()
                        .strafeTo(Vector2d(41.0, -24.5))
                        .build()
        )

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
                ReflectiveMenuItemBoolean("Foundation Swivel", ::foundationSwivel)
//                ReflectiveMenuItemEnum("OpenCV Detector", ::visionDetector, *PixelStatsPipeline.StatsDetector.values()),
//                ReflectiveMenuItemBoolean("Foundation Redundancy", ::foundationRedundancy),
//                ReflectiveMenuItemBoolean("Push Alliance Partner", ::pushAlliancePartner),
//                ReflectiveMenuItemBoolean("Park Only", ::parkOnly),
//                ReflectiveMenuItemInteger("Parking Delay", ::delayBeforeParking, 0, 25, 1),
//                ReflectiveMenuItemBoolean("Park Near DS", ::parkNearDS),
//                ReflectiveMenuItemBoolean("Park After Task", ::parkAfterTask)
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
        }
    }


    /**
     * Robot strafes for desired distance until motors have reached target.
     * Forwards distances and power to [robot.holonomic]
     */
    private fun drive(x: Double, y: Double, power: Double) {
        robot.holonomic.runUsingEncoder(x, y, power)
        val originalRuntime = runtime
        @Suppress("StatementWithEmptyBody") while (opModeIsActive() && robot.holonomic.motorsAreBusy() && runtime - originalRuntime < 3);
    }

    /**
     * Robot strafes with specified power until desired time length is reached.
     * Forwards power to [robot.holonomic]
     */
    private fun timeDrive(x: Double, y: Double, z: Double, timeMs: Long) {
        robot.holonomic.runWithoutEncoder(x, y, z)
        sleep(timeMs)
        robot.holonomic.stop()

    }

    /**
     * Lifts the intake arm and then holds it with constant power
     */
    private fun doArmLift(target: Double) {
        var currentValue = robot.intakePivotPotentiometer.voltage
        val kP = 3.5
        val originalRuntime = runtime

        while (currentValue > target && opModeIsActive() && runtime - originalRuntime < 1) {
            robot.intakePivotMotor.power = -kP * target - currentValue
            telem.update()
        }
        robot.intakePivotMotor.power = 0.12
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

    private inline fun positionalConstructor() = robot.positionalHolonomic.positionConstructor()
}