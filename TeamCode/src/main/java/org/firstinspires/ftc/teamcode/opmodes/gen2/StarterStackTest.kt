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
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingTapper
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingContourPipeline
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DEMO: Starter Stack Collection", group = "Main")
class StarterStackTest : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtZoomBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private var cvContainer     : OpenCvContainer<RingContourPipeline>? = null

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
//        cvContainer = VisionFactory.createOpenCv(
//                VisionFactory.CameraType.WEBCAM_MINUS,
//                hardwareMap,
//                RingContourPipeline())
        cvContainer?.pipeline?.shouldKeepTracking = true
        cvContainer?.pipeline?.tracking = true
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

        val numRings = cvContainer?.pipeline?.numberOfRings
        cvContainer?.pipeline?.tracking = false
        telemetry.addData("Number of rings", numRings)
        telemetry.addData("Ratio", cvContainer?.pipeline?.ratio)
        telemetry.update()

        doFullAuto(0)
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
        for (i in 1..3) {
            robot.intakeStage1.power = ExtZoomBotConstants.AUTO_INTAKE_POWER_1
            robot.intakeStage2.power = ExtZoomBotConstants.AUTO_INTAKE_POWER_2
            timeDrive(0.0, ExtZoomBotConstants.AUTO_INTAKE_DRIVE_SPEED, 0.0, ExtZoomBotConstants.AUTO_INTAKE_TIME.toLong())
            timeDrive(0.0, ExtZoomBotConstants.AUTO_INTAKE_DRIVE_SPEED_REV, 0.0, ExtZoomBotConstants.AUTO_INTAKE_TIME_REV.toLong())
            robot.intakeStage1.power = 0.0
            robot.ringTapIntoMagazine.move(RingTapper.Position.TAP)
            sleep(200)
            robot.ringTapIntoMagazine.move(RingTapper.Position.STORAGE)
        }


        doShootThreeRings()


    }

    /**
     * Shoots three rings
     * @param cutoff Whether or not to disable flywheel after ring shooting
     */
    fun doShootThreeRings(cutoff: Boolean = true) {
        robot.zoomWheel.velocity = 1100.0
        sleep(1500)

        for (i in 1..3) {
            hit()
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

    fun hit() {
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
        sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
        sleep(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong())
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
            if (this@StarterStackTest.allianceColor==testColor) -this else this

    /**
     * Reverses input number if [testLine] matches [startingLine]
     */
    private infix fun Double.reverseIf(testLine: StartingLine): Double =
            if (this@StarterStackTest.startingLine==testLine) -this else this

    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}