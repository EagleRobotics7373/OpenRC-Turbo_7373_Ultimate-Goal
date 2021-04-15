package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingLine.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsRingPlace
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingTapper
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingContourPipeline
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig
import kotlin.math.absoluteValue

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous State (Kotlin + RR)", group = "Main")
class AutonomousRRState : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtZoomBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<RingContourPipeline>

    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    private val config = OpModeConfig(telemetry)
    private var allianceColor: AllianceColor by config.custom("Alliance Color", BLUE)
    private var startingLine: StartingLine by config.custom("Starting Line", FAR)
//    private var endingRegion: StartingLine by config.custom("Ending Region", FAR, CENTER)
//    private var delayBeforeStart: Int by config.int("Delay Before Start", 0, 0..10 step 1)
//    private var delayBeforePark: Int by config.int("Delay Before Park", 7, 0..10 step 1)
    private var sleepBeforeShoot: Int by config.int("Delay Before Shoot (ms)", 400, 0..1000 step 100)
    private var intoSquareDisp: Int by config.int("Target Zone Displacement (in)", -2, -4..7);
    private var wobbleYDisp: Int by config.int("Wobble Y Displacement (in)", 0, -4..4);
    private var idealDistFromWobble: Int by config.int("Ideal Dist From Wobble", 5, 2..9);
    private var collectStarterStack: Boolean by config.boolean("Collect Starter Stack", true);
    private var singleRingIntoPowerShot: Position by config.custom("Single Ring Into Power Shot", Position.RIGHT, Position.CENTER, Position.NULL);

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = ExtZoomBot(hardwareMap)
        imuController = robot.imuControllerC
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM_MINUS,
                hardwareMap,
                RingContourPipeline())
        cvContainer.pipeline.shouldKeepTracking = true
        cvContainer.pipeline.tracking = true
        robot.zoomWheel.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.zoomWheel.velocity = 0.0



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
        robot.holonomicRR.redefine()
        robot.wobbleGrabberSide.state = WobbleGrabber.WobbleGrabberState.STORAGE
        robot.ringTapIntoMagazine.move(RingTapper.Position.STORAGE)


        elapsedTime = ElapsedTime()



        /*
            Perform actions
         */

        val numRings = cvContainer.pipeline.numberOfRings
        cvContainer.pipeline.tracking = false
        telemetry.addData("Number of rings", numRings)
        telemetry.addData("Ratio", cvContainer.pipeline.ratio)
        telemetry.update()

        doFullAuto(numRings ?: 0)

        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

//        Thread {
//            cvContainer.stop()
//            cvContainer.camera.closeCameraDevice()
//        }.start()

    }

    fun doFullAuto(numRings: Int) {
//        sleep(delayBeforeStart.toLong().times(1000))
//        thread { robot.intakeSystem.update() }
        // Create a static map of wobble goal drop-off positions for each number of rings
        val farStartY = 57.0
        val wobbleDropoffs = mapOf(
                0 to Vector2d(-6.5 + intoSquareDisp, -farStartY reverseIf BLUE),
                1 to Vector2d(16.0 + intoSquareDisp, -36.0 reverseIf BLUE),
                4 to Vector2d(43.0 + intoSquareDisp, -60.0 reverseIf BLUE)
        )
        val secondaryWobbleDropoffs = wobbleDropoffs.mapValues {
            Vector2d(
                    x = it.value.x + 18.0 + if (it.key == 0) 4.0 else 3.0,
                    y = it.value.y + (18.0 reverseIf BLUE)
            )
        }

        // Set the robot starting position within RoadRunner
        robot.holonomicRR.poseEstimate = Pose2d(
                -63.0,
                (if (startingLine == CENTER) -24.0 else -farStartY) reverseIf BLUE,
                PI
        )

        robot.zoomWheel.velocity = 1175.0
        robot.deflectionServo.position = ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_AUTO_DIAG

        // Do the initial movement to wobble drop-off location. We will shoot 3x rings from here
        builder(0.0)
                .splineToConstantHeading(
                        endPosition = wobbleDropoffs[0] ?: error("Can't find wobble target #0 in wobbleDropoffs"),
                        endTangent = 0.0,
                        constraintsOverride =
                                DriveConstraints(
                                        80.0, 80.0, 80.0,
                                        PI, PI, 0.0)
                )
                .buildAndRun()

        // Now we will shoot three rings, with deflector
        if (numRings == 0) robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.VERTICAL)
        doShootThreeRings(speed = 1175.0, wait = false)
        if (numRings == 0) doWobblePlace(robot.wobbleGrabber)
        robot.deflectionServo.position = ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_STORED

        // Strafe towards the starter stack.
        builder(-PI)
                .strafeTo(
                        endPosition = Vector2d(
                                x = -12.0,
                                y = 37.0 reverseIf RED
                        )
                )
                .buildAndRun()

        if (collectStarterStack && numRings > 0) {

            if (numRings != 1) singleRingIntoPowerShot = Position.NULL

            robot.deflectionServo.position = when(singleRingIntoPowerShot) {
                Position.CENTER -> ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_AUTO_DIAG_SHOT_CENTER
                Position.RIGHT  -> ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_AUTO_DIAG_SHOT_RIGHT
                else            -> ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_AUTO_DIAG_SLIM
            }

            robot.zoomWheel.velocity = when(singleRingIntoPowerShot) {
                Position.CENTER, Position.RIGHT -> 1100.0
                else                            -> 1125.0
            }

            val numRingsToIntake = if (numRings == 1) 1 else 3
            for (i in 1..numRingsToIntake) {
                robot.intakeStage1.power = ExtZoomBotConstants.AUTO_INTAKE_POWER_1
                robot.intakeStage2.power = ExtZoomBotConstants.AUTO_INTAKE_POWER_2
                timeDrive(0.0, ExtZoomBotConstants.AUTO_INTAKE_DRIVE_SPEED, 0.0, ExtZoomBotConstants.AUTO_INTAKE_TIME.toLong())
                timeDrive(0.0, ExtZoomBotConstants.AUTO_INTAKE_DRIVE_SPEED_REV, 0.0, ExtZoomBotConstants.AUTO_INTAKE_TIME_REV.toLong())
                robot.intakeStage1.power = 0.0
                robot.ringTapIntoMagazine.move(RingTapper.Position.TAP)
                sleep(
                        (if (numRingsToIntake == 1) ExtZoomBotConstants.AUTO_INTAKE_TAP_SLEEP_SHORT
                        else ExtZoomBotConstants.AUTO_INTAKE_TAP_SLEEP_SHORT).toLong()
                )
                robot.ringTapIntoMagazine.move(RingTapper.Position.STORAGE)
            }

            robot.intakeStage2.power = 0.0

            builder()
                    .strafeTo(
                            endPosition = Vector2d(
                                    x = -3.0,
                                    y = 37.0
                            ),
                            constraintsOverride = DriveConstraints(
                                    30.0, 25.0, 40.0,
                                    PI, PI, 0.0)
                    )
                    .buildAndRun()

            if (numRingsToIntake == 3) doShootThreeRings(speed = null, wait = false, cutoff = false)
            else hit()

            robot.zoomWheel.velocity = 0.0
        }

        // Drive to targets #1 or #4, if applicable, to place first wobble goal
        if (numRings != 0) {
            robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.VERTICAL)
            builder(tangent = 0.0)
                    .splineToConstantHeading(
                            endPosition = wobbleDropoffs[numRings] ?: error("numRings=$numRings is invalid in wobbleDropoffs"),
                            endTangent = if (numRings == 1) 0.0 else PI / 4 reverseIf RED,
                            constraintsOverride = DriveConstraints(
                                    70.0, 60.0, 50.0,
                                    PI, PI, 0.0)
                    )
                    .buildAndRun()
            doWobblePlace(robot.wobbleGrabber)
        }

        // Create Vector2d position representing robot next to wobble goal
        val secondWobbleVector =
                Vector2d(
                        x = -43.5,
                        y = 8.0 + wobbleYDisp reverseIf RED
                )
        // Create Vector2d position representing middle of path to second wobble goal
        val wobbleArcMidVector = Vector2d(
                robot.holonomicRR.poseEstimate.x,
                (robot.holonomicRR.poseEstimate.y - 0) / 2 reverseIf RED
        )

        if (numRings == 0) {
            // Strafe to second wobble start
            builder(PI.div(2) reverseIf BLUE)
                    .splineToConstantHeading(
                            endPosition = wobbleArcMidVector,
                            endTangent = -PI / 2 reverseIf RED
                    )
                    .splineToConstantHeading(
                            endPosition = secondWobbleVector,
                            endTangent = -PI
                    )
                    .buildAndRun()
        } else {
            // Strafe to second wobble start
            builder(-PI / 2 reverseIf RED)
                    .splineToConstantHeading(
                            endPosition = Vector2d(-6.5, secondWobbleVector.y),
                            endTangent = PI reverseIf RED
                    )
                    .splineToConstantHeading(
                            endPosition = secondWobbleVector,
                            endTangent = -PI
                    )
                    .addDisplacementMarker(
                            0.1,
                            0.0,
                            MarkerCallback { robot.wobbleGrabberSide.move(pivot = WobbleGrabber.PivotPosition.VERTICAL)})
                    .buildAndRun()
        }

        val currentDistFromWobble = doGetCurrentDistFromWobble()
        val distanceToTravel = currentDistFromWobble - idealDistFromWobble + 0.5

        telemetry.addData("Current Dist from Wobble", currentDistFromWobble)
        telemetry.addData("Distance to Travel Right", distanceToTravel)
        if (distanceToTravel.absoluteValue > 0.5)
            builder()
                    .strafeRight(distanceToTravel)
                    .buildAndRun()
        telemetry.update()

        // Pick up second wobble goal here
        doWobblePickup(robot.wobbleGrabberSide)

        if (numRings == 0) {
            // Strafe to target zone #0
            builder(0.0)
                    .splineToConstantHeading(
                            endPosition = Vector2d(
                                    wobbleArcMidVector.x,
                                    secondWobbleVector.y
                            ),
                            endTangent = 0.0)
                    .splineToConstantHeading(
                            endPosition = secondaryWobbleDropoffs[numRings] ?: error("Incorrect number of rings set"),
                            endTangent = PI / 4 reverseIf RED
                    )
                    .buildAndRun()
        } else {
            // Strafe to target zone #1 or #4
            builder(0.0)
                    .splineToConstantHeading(
                            endPosition = Vector2d(
                                    0.0,
                                    secondWobbleVector.y
                            ),
                            endTangent = 0.0,
                            constraintsOverride = DriveConstraints(
                                    80.0, 70.0, 50.0,
                                    PI, PI, 0.0))
                    .splineToConstantHeading(
                            endPosition = secondaryWobbleDropoffs[numRings] ?: error("Incorrect number of rings set"),
                            endTangent = (PI / 4) reverseIf RED,
                            constraintsOverride = DriveConstraints(
                                    60.0, 50.0, 50.0,
                                    PI, PI, 0.0)
                    )
                    .buildAndRun()
        }

        doWobblePlace(robot.wobbleGrabberSide)

        if (numRings != 0){
            // Strafe to center line
            builder()
                    .strafeTo(
                            endPosition = Vector2d(
                                    x = 12.0,
                                    y = robot.holonomicRR.poseEstimate.y
                            ),
                            constraintsOverride = DriveConstraints(
                                    80.0, 90.0, 50.0,
                                    PI, PI, 0.0)
                    )
                    .buildAndRun()
        }


    }

    /**
     * Shoots three rings
     * @param cutoff Whether or not to disable flywheel after ring shooting
     */
    fun doShootThreeRings(speed: Double? = 1100.0, wait: Boolean = true, cutoff: Boolean = true) {
        if (speed != null) robot.zoomWheel.velocity = speed
        if (wait) sleep(sleepBeforeShoot.toLong())

        for (i in 1..3) {
            hit()
        }

        if (cutoff) robot.zoomWheel.velocity = 0.0
    }

    fun hit() {
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
        sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
        sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
    }

    fun doWobblePlace(wobbleGrabber: WobbleGrabber) {
        // Drop wobble goal after rings shot
        wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.PERPENDICULAR)
        sleep(700)
        wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.STORAGE)
        sleep(400)
        wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.VERTICAL)

    }

    fun doWobblePickup(wobbleGrabber: WobbleGrabber) {
        // Drop wobble goal after rings shot
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.GRAB_PREP
        sleep(1200)
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.GRAB
        sleep(600)
        wobbleGrabber.nextState()
        sleep(600)
    }

    fun doGetCurrentDistFromWobble(): Double {
        var accumulatedSum = 0.0
        val count = 10
        for (i in 1..count) {
            accumulatedSum += robot.sideWobbleDistSensor.getDistance(DistanceUnit.INCH)
            sleep(25)
        }
        return accumulatedSum / count
    }

    /**
     * Creates and operates [ReflectiveTelemetryMenu] before the init period.
     * Controls code until [isStopRequested] or [isStarted] is true.
     */

    fun operateMenu() {

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        config.update()

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.invoke()    -> config.update(prevItem = true)
                dpadDownWatch.invoke()  -> config.update(nextItem = true)
                dpadLeftWatch.invoke()  -> config.update(iterBack = true)
                dpadRightWatch.invoke() -> config.update(iterFw   = true)
            }

            when {
                gamepad1.x -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.GRAB)
                gamepad1.y -> {
                    robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.YEET)
                    robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.MID_GRAB)
                }
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

    private fun forDurationMs(duration: Long, action: ()->Unit) {
        val end = System.currentTimeMillis() + duration
        while (System.currentTimeMillis() < end) action.invoke()
    }

    /**
     * Reverses input number if [testColor] matches [allianceColor]
     */
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double =
            if (this@AutonomousRRState.allianceColor==testColor) -this else this

    /**
     * Reverses input number if [testLine] matches [startingLine]
     */
    private infix fun Double.reverseIf(testLine: StartingLine): Double =
            if (this@AutonomousRRState.startingLine==testLine) -this else this

    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}