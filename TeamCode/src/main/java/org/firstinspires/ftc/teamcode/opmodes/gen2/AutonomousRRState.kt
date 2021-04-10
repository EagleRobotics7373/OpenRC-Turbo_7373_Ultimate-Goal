package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingLine.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingContourPipeline
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig

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
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingLine: StartingLine by config.custom("Starting Line", FAR, CENTER)
    private var endingRegion: StartingLine by config.custom("Ending Region", FAR, CENTER)
    private var delayBeforeStart: Int by config.int("Delay Before Start", 0, 0..10 step 1)
    private var delayBeforePark: Int by config.int("Delay Before Park", 7, 0..10 step 1)
    private var sleepBeforeShoot: Int by config.int("Delay Before Shoot (ms)", 0, 0..1000 step 100)

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


        elapsedTime = ElapsedTime()



        /*
            Perform actions
         */

        val numRings = cvContainer.pipeline.numberOfRings
        cvContainer.pipeline.tracking = false
        telemetry.addData("Number of rings", numRings)
        telemetry.addData("Ratio", cvContainer.pipeline.ratio)
        telemetry.update()

        doFullAuto(numRings)

        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

//        Thread {
//            cvContainer.stop()
//            cvContainer.camera.closeCameraDevice()
//        }.start()

    }

    fun doFullAuto(numRings: Int?) {
        sleep(delayBeforeStart.toLong().times(1000))
//        thread { robot.intakeSystem.update() }
        // Create a static map of wobble goal drop-off positions for each number of rings
        val intoSquareDisp = 6.0
        val farStartY = 57.0
        val wobbleDropoffs = mapOf(
                0 to Vector2d(-6.5 + intoSquareDisp, -farStartY reverseIf BLUE),
                1 to Vector2d(16.0 + intoSquareDisp, -36.0 reverseIf BLUE),
                4 to Vector2d(40.0 + intoSquareDisp, -60.0 reverseIf BLUE)
        )
        val secondaryWobbleDropoffs = wobbleDropoffs.map {
            Vector2d(
                    x = it.value.x + 18.0 + if (it.key == 0) 4.0 else 0.0,
                    y = it.value.y + (24.0 reverseIf BLUE)
            )
        }

        // Set the robot starting position within RoadRunner
        robot.holonomicRR.poseEstimate = Pose2d(
                -63.0,
                (if (startingLine == CENTER) -24.0 else -farStartY) reverseIf BLUE,
                0.0
        )

        // Do the initial movement to wobble drop-off location. We will shoot 3x rings from here
        builder(0.0)
                .splineToConstantHeading(
                        endPosition = Vector2d(
                                x = -24.0,
                                y = wobbleDropoffs[0]?.y ?: error("Didn't create valid wobble drop-off positions pre-compilation")
                        ),
                        endTangent = 0.0
                )
                .splineToConstantHeading(
                        endPosition = wobbleDropoffs[0] ?: error("Incorrect number of rings set"),
                        endTangent = if (startingLine == CENTER) -PI.div(2) reverseIf BLUE else 0.0
                )
                .buildAndRun()

        // Now we will shoot three rings, with deflector
        robot.deflectionServo.position = ExtZoomBotConstants.SERVO_DEFLECTION_POS_DEFAULT_AUTO_DIAG
        doShootThreeRings()
        robot.zoomWheel.power = 0.0

        if (numRings == 0) doWobblePlace(robot.wobbleGrabber)

        // Drive in front of the starter stack
        builder(-PI / 2 reverseIf RED)
                .strafeTo(
                        endPosition = Vector2d(
                                x = -3.0,
                                y = 37.0 reverseIf RED
                        )
                )
                .buildAndRun()

        // Strafe backwards as if we're collecting the starter stack.
//        builder(-PI)
//                .strafeTo(
//                        endPosition = Vector2d(
//                                x = -12.0,
//                                y = 37.0 reverseIf RED
//                        )
//                )
//                .buildAndRun()

        // Drive to targets #1 or #4, if applicable, to place first wobble goal
        if (numRings != 0) {
            builder(tangent = 0.0)
                    .splineToConstantHeading(
                            endPosition = wobbleDropoffs[numRings] ?: error("Incorrect number of rings set"),
                            endTangent = if (numRings == 1) 0.0 else PI / 4 reverseIf RED
                    )
                    .buildAndRun()
            doWobblePlace(robot.wobbleGrabber)
        }

        // Create Vector2d position representing robot next to wobble goal
        val secondWobbleVector =
                Vector2d(
                        x = -43.0,
                        y = 6.0 reverseIf RED
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
                            0.9,
                            0.0,
                            MarkerCallback { robot.wobbleGrabberSide.state = WobbleGrabber.WobbleGrabberState.RELEASE})
                    .buildAndRun()
        }

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
                            endTangent = 0.0)
                    .splineToConstantHeading(
                            endPosition = secondaryWobbleDropoffs[numRings!!],
                            endTangent = (if (numRings == 1) PI / 4 else PI / 2) reverseIf RED
                    )
                    .buildAndRun()
        }

        if (numRings != 0){
            // Strafe to center line
            builder()
                    .strafeTo(
                            endPosition = Vector2d(
                                    x = 12.0,
                                    y = robot.holonomicRR.poseEstimate.y
                            )
                    )
                    .buildAndRun()
        }


    }

    /**
     * Shoots three rings
     * @param cutoff Whether or not to disable flywheel after ring shooting
     */
    fun doShootThreeRings(cutoff: Boolean = true) {
        robot.zoomWheel.velocity = 1100.0
        sleep(sleepBeforeShoot.toLong())

        for (i in 1..3) {
            robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
            sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
            robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
            sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
        }

        if (cutoff) robot.zoomWheel.velocity = 0.0
    }

    fun doWobblePlace(wobbleGrabber: WobbleGrabber) {
        // Drop wobble goal after rings shot
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.GRAB
        sleep(1000)
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.RELEASE
    }

    fun doWobblePickup(wobbleGrabber: WobbleGrabber) {
        // Drop wobble goal after rings shot
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.GRAB_PREP
        sleep(700)
        wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.GRAB
        sleep(700)
        wobbleGrabber.nextState()
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