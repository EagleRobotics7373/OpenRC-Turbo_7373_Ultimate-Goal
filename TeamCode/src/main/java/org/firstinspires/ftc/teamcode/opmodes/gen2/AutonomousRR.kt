package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous LT (Kotlin + RR)", group = "Main")
class AutonomousRR : LinearOpMode() {

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
    private var darBots: Boolean by config.boolean("Darbots Mode", false)

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
        val wobbleDropoffs = mapOf(
                0 to Vector2d(-6.5 + intoSquareDisp, -60.0 reverseIf BLUE),
                1 to Vector2d(16.0 + intoSquareDisp, -36.0 reverseIf BLUE),
                4 to Vector2d(40.0 + intoSquareDisp, -60.0 reverseIf BLUE)
        )

        // Set the robot starting position within RoadRunner
        robot.holonomicRR.poseEstimate = Pose2d(
                -63.0,
                (if (startingLine == CENTER) -24.0 else -48.0) reverseIf BLUE,
                PI
        )

        // Do the initial movement to wobble drop-off location
        builder(PI.div(4) reverseIf BLUE reverseIf FAR)
                // Spline to drive around and preserve the starter stack
                .splineToConstantHeading(
                        endPosition = Vector2d(
                                x = -24.0,
                                y = robot.holonomicRR.poseEstimate.y.plus(6.0 reverseIf FAR reverseIf BLUE)
                        ),
                        endTangent = 0.0
                )
                // Spline to the wobble drop-off position
                .splineToConstantHeading(
                        endPosition = wobbleDropoffs[numRings] ?: error("Incorrect number of rings set"),
                        endTangent = if (startingLine == CENTER) -PI.div(2) reverseIf BLUE else 0.0
                )
                .buildAndRun()

        robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.PERPENDICULAR)
        sleep(1500)
        robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.STORAGE)
        sleep(500)
        robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.VERTICAL)
        sleep(2500)

        val shootVector = Vector2d(-3.0, if (allianceColor == BLUE) 34.5 else -37.0)
        robot.zoomWheel.velocity = 1100.0

        if (numRings == 0) {
            builder(PI.div(2) reverseIf BLUE)
                    .splineToConstantHeading(
                            endPosition = shootVector,
                            endTangent = 0.0
                    )
                    .buildAndRun()
        } else {
            builder()
                    .strafeTo(shootVector)
                    .buildAndRun()
        }

//        robot.zoomWheel.velocity = 1100.0
        sleep(sleepBeforeShoot.toLong())

        for (i in 1..3) {
            robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
            sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
            robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
            sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
        }
        robot.zoomWheel.velocity = 0.0

        if (darBots) {

            builder(tangent = (-PI/2) reverseIf RED)
                    .splineToConstantHeading(Vector2d(11.0, (-11.0) reverseIf BLUE), 0.0)
                    .splineToConstantHeading(Vector2d(58.0, (-11.0) reverseIf BLUE), 0.0)
                    .buildAndRun()

            val runtime = System.currentTimeMillis()
            val beforePark = runtime + delayBeforePark * 1000

            while (System.currentTimeMillis() < beforePark && opModeIsActive());
        }

        builder()
                .strafeTo(Vector2d(11.0, (if (endingRegion == CENTER) 10.0 else 37.0) reverseIf RED))
                .buildAndRun()

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
            if (this@AutonomousRR.allianceColor==testColor) -this else this

    /**
     * Reverses input number if [testLine] matches [startingLine]
     */
    private infix fun Double.reverseIf(testLine: StartingLine): Double =
            if (this@AutonomousRR.startingLine==testLine) -this else this

    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}