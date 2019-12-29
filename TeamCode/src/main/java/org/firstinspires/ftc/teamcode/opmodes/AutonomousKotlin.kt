package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.Position.*
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer
import org.firstinspires.ftc.teamcode.opmodes.control.IMUPIDStrafer
import kotlin.math.absoluteValue

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous (Kotlin)", group = "Main")
class AutonomousKotlin : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : BasicRobot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<PixelStatsPipeline>

    /*
        VARIABLES: Menu Options
     */
                     var allianceColor                       = RED
                     var startingPosition                    = FieldSide.WAFFLE_SIDE
                     var musicFile                           = ExtMusicFile.NONE
                     var parkOnly                            = false
                     var visionDetector                      = PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV
                     var delayBeforeParking                  = 0
                     var parkNearDS                          = false
                     var parkAfterTask                       = true
                     var foundationRedundancy                = true
                     var pushAlliancePartner                 = true

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = BasicRobot(hardwareMap)
        imuController = IMUController(hardwareMap)
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                PixelStatsPipeline(PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


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
        val player = ExtDirMusicPlayer(musicFile)
        player.play()

        elapsedTime = ElapsedTime()

        cvContainer.pipeline.detector = visionDetector


        /*
            Perform actions
         */
        if (parkOnly) doParkOnlyAuto()
        else when (startingPosition) {
            FieldSide.WAFFLE_SIDE  -> doWaffleSideAuto()
            FieldSide.LOADING_ZONE -> doLoadingZoneAuto()
        }


        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive());

        cvContainer.stop()
        player.stop()
    }

    fun doParkOnlyAuto() {
        sleep(delayBeforeParking * 1000.toLong())
        drive(0.0, 12.0, 0.5)
    }

    fun doWaffleSideAuto() {
        // Since we can't start inside of the building site, drive into the building site.
        drive(24.0 reverseIf BLUE, 0.0, 0.8)
        sleep(250)

        // Drive to the foundation and hold foundation
        drive(0.0, 29.0, 0.4)
        robot.foundationGrabbers.lock()
        sleep(1000)

        // Drive back to the wall and release foundation grabbers
        timeDrive(0.0, -0.5, 0.0, 2000)
        robot.foundationGrabbers.unlock()
        sleep(500)

        // Push foundation and park if parkAfterTask is true
        if (parkAfterTask) {

            // Drive against the wall towards the bridge, but not under it
            drive(-35.0 reverseIf BLUE, 0.0, 0.2)

            if (foundationRedundancy) {
                // Drive parallel to the alliance bridge, then drive toward the foundation.
                // This ensures that the foundation is fully scored in the building site.
                drive(0.0, 17.0, 0.2)
                drive(20.0 reverseIf BLUE, 0.0, 0.2)
                sleep(500)

                // Drive under the alliance bridge
                drive(-26.0 reverseIf BLUE, 0.0, 0.2)
            } else {
                // Since foundationRedundancy is false, drive under the bridge, then away from driver station
                drive(-16.0 reverseIf BLUE, 0.0, 0.5)
                drive(0.0, 8.0, 0.4)
            }

            // Drive towards or away from the driver station, allowing room for another robot to park.
            if (parkNearDS) drive(0.0, -24.0, 0.2)
            else        timeDrive(0.0, 0.4, 0.0, 500)
        }
    }

    fun doLoadingZoneAuto() {
        // Lift intake arm so camera has open view, then enable SKYSTONE detection
        doArmLift(1.153)
        cvContainer.pipeline.tracking = true
        while (cvContainer.pipeline.tracking) sleep(50)

        // Obtain SKYSTONE position after tracking has finished
        val skystonePosition = cvContainer.pipeline.skystonePos

        // Reset the x-axis odometry counter, then drive towards the stones
        robot.odometryXAxis.resetHWCounter()
        drive(0.0, 26.0, 0.4)
        sleep(500)

        // Translate LEFT/RIGHT detector result to field-oriented stone position
        val translatedSkystonePosition =
                when (skystonePosition) {
                    LEFT  -> CENTER
                    RIGHT -> RIGHT
                    else  -> LEFT
                }

        // Set upcoming movement targets based on translated SKYSTONE position
        var crossFieldStrafeTarget = 68.0
        val distanceBetweenSkystones = 8.0
        val stoneStrafeTarget : Double
        when (allianceColor) {
            RED ->
                when (translatedSkystonePosition) {
                    LEFT   -> {
                        crossFieldStrafeTarget +=  2 * distanceBetweenSkystones
                        stoneStrafeTarget      =  -distanceBetweenSkystones - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                    CENTER -> {
                        crossFieldStrafeTarget +=  1 * distanceBetweenSkystones
                        stoneStrafeTarget      =  -1                        - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                    else  -> {
                        crossFieldStrafeTarget +=  0 * distanceBetweenSkystones
                        stoneStrafeTarget      =   distanceBetweenSkystones - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                }
            BLUE ->
                when (translatedSkystonePosition) {
                    LEFT   -> {
                        crossFieldStrafeTarget +=  0 * distanceBetweenSkystones
                        stoneStrafeTarget      =  -distanceBetweenSkystones - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                    CENTER -> {
                        crossFieldStrafeTarget +=  1 * distanceBetweenSkystones
                        stoneStrafeTarget      =  -1                        - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                    else  -> {
                        crossFieldStrafeTarget +=  2 * distanceBetweenSkystones
                        stoneStrafeTarget      =   distanceBetweenSkystones - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH)
                    }
                }
        }

        robot.odometryXAxis.resetHWCounter()

        // Strafe toward SKYSTONE
        IMUPIDStrafer(
                robot.holonomic, imuController,
                PIDCoefficients(0.2, 0.0000013, 0.0), PIDCoefficients(1.0, 0.0, 0.0))
                { stoneStrafeTarget - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH) }

                .run {
                    // Force max power of 0.2
                    setStartingLimit(19000, 0.2)
                    // Use strafer while not at target and within runtime
                    while (strafeErrorFun().absoluteValue > 0 && opModeIsActive() && System.currentTimeMillis() - startingRuntime < 3000)
                        run()
                    robot.holonomic.stop()
                }

        // Slowly let intake arm fall, then apply constant power
        while (opModeIsActive() && robot.intakePivotPotentiometer.voltage < 1.6)
            robot.intakePivotMotor.power = 0.01
        robot.intakePivotMotor.power = 0.12

        // Drive forward, then grab SKYSTONE, letting arm fall
        drive(0.0, 7.0, 0.2)
        robot.intakeBlockGrabber.hold()
        robot.intakeBlockManipulator.power = 1.0
        robot.intakePivotMotor.power = 0.0

        // Wait for servo movement, then drive backwards with stone
        sleep(500)
        drive(0.0, -8.5, 0.3)

        // Strafe with SKYSTONE across the field, towards foundation
        robot.odometryXAxis.resetHWCounter()
        IMUPIDStrafer(
                robot.holonomic, imuController,
                PIDCoefficients(0.55, 0.0, 0.0), PIDCoefficients(1.75, 0.0, 0.0))
                { 1.0 reverseIf BLUE }

                .run {
                    // Use strafer while more than 5 inches away from target
                    while (crossFieldStrafeTarget.absoluteValue - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH) > 5 && opModeIsActive())
                        run()

                    // Once robot is 5 inches away from target, raise/lower intake arm to desired position
                    Thread {
                        if (robot.intakePivotPotentiometer.voltage < 1.366) {
                            robot.intakePivotMotor.power = 0.01
                            while (robot.intakePivotPotentiometer.voltage < 1.366);
                        } else
                            doArmLift(1.366)
                        robot.intakePivotMotor.power = 0.12
                    }.start()

                    // Lower strafe power to 0.2 and continue strafing until robot is 1 inch away from target
                    strafePIDCoefficients.p = 0.2
                    while (crossFieldStrafeTarget.absoluteValue - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH) > 5 && opModeIsActive())
                        run()

                    // Stop robot once target has been reached
                    robot.holonomic.stop()
                }

        // Drive to foundation and grab it
        drive(0.0, 11.0, 0.5)
        robot.foundationGrabbers.lock()

        // Wait for foundation grabbers, then release SKYSTONE onto foundation
        sleep(1500)
        robot.intakeBlockGrabber.release()
        robot.intakeBlockManipulator.power = -1.0

        // Wait for stone to drop, then stop intake motor and riase arm
        sleep(1500)
        robot.intakeBlockManipulator.power = 0.0
        Thread { doArmLift(1.1) }.start()

        // Pull foundation into building site, then release foundation grabbers
        timeDrive(0.0, -0.5, 0.0, 2000)
        robot.foundationGrabbers.unlock()

        // Strafe toward bridge for parking and/or push alliance partner under bridge
        robot.odometryXAxis.resetHWCounter()
        val parkingStrafeTarget = (if (pushAlliancePartner) 27.0 else 26.0) reverseIf RED
        IMUPIDStrafer(
                robot.holonomic, imuController,
                PIDCoefficients(0.005, 0.0000022, 0.0), PIDCoefficients(1.75, 0.0, 0.0))
                { parkingStrafeTarget - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH) }

                .run {
                    // Use strafer while more than 1 inch away from target
                    while (strafeErrorFun().absoluteValue > 1 && opModeIsActive())
                        run()

                    // Stop robot once target has been reached, and sleep
                    robot.holonomic.stop()
                    sleep(500)
                }

        // If robot pushed alliance partner, drive away from alliance partner
        if (pushAlliancePartner) {
            IMUPIDStrafer(
                    robot.holonomic, imuController,
                    PIDCoefficients(0.25, 0.0, 0.0), PIDCoefficients(1.75, 0.0, 0.0))
                    { 1.0 reverseIf BLUE }

                    .run {
                        // Set constant input of -0.25 on y-axis for staying close to wall
                        yPower = -0.25

                        // Use strafer while away from target
                        while (1.35 - robot.odometryXAxis.getDistanceNormalized(DistanceUnit.INCH).absoluteValue > 0 && opModeIsActive())
                            run()

                        // Stop robot once target has been reached
                        robot.holonomic.stop()
                    }
        }

        // Let intake arm lower completely
        robot.intakePivotMotor.power = 0.0

        // Straighten back against field wall, then drive away from wall
        timeDrive(0.0, -0.4, 0.0, 500)
        drive(0.0, 27.0, 0.5)

        // Drive under alliance bridge, earning parking bonus
        drive(-18.0 reverseIf BLUE, 0.0, 0.5)

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
                ReflectiveMenuItemEnum("OpenCV Detector", ::visionDetector, *PixelStatsPipeline.StatsDetector.values()),
                ReflectiveMenuItemBoolean("Foundation Redundancy", ::foundationRedundancy),
                ReflectiveMenuItemBoolean("Push Alliance Partner", ::pushAlliancePartner),
                ReflectiveMenuItemBoolean("Park Only", ::parkOnly),
                ReflectiveMenuItemInteger("Parking Delay", ::delayBeforeParking, 0, 25, 1),
                ReflectiveMenuItemBoolean("Park Near DS", ::parkNearDS),
                ReflectiveMenuItemBoolean("Park After Task", ::parkAfterTask)
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
}