package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import android.support.annotation.NonNull
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstants.*

class HolonomicRR

constructor (hardwareMap: HardwareMap,
             frontLeftMotor: DcMotor,
             backLeftMotor:  DcMotor,
             backRightMotor: DcMotor,
             frontRightMotor:DcMotor,
             leftOdometryModule: DcMotor,
             rightOdometryModule: DcMotor,
             rearOdometryModule: DcMotor,
             val imuController: IMUController)

    : MecanumDrive(kV, kA, kStatic, TRACK_WIDTH) // TODO: Define these variables
{

    private val frontLeftExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(frontLeftMotor).first())
    private val backLeftExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(backLeftMotor).first())
    private val backRightExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(backRightMotor).first())
    private val frontRightExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(frontRightMotor).first())

    private val motorsHubExt = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")
    private val odometryHubExt = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")

    private val odometryLeftExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(leftOdometryModule).first())
    private val odometryRightExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(rightOdometryModule).first())
    private val odometryRearExt = hardwareMap.get(ExpansionHubMotor::class.java, hardwareMap.getNamesOf(rearOdometryModule).first())

    val motors    = listOf(frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor)
    val motorsExt = listOf(frontLeftExt, backLeftExt, backRightExt, frontRightExt)

    enum class Mode { IDLE, TURN, FOLLOW_TRAJECTORY }
    var mode = Mode.IDLE

    val clock = NanoClock.system()
    val dashboard = FtcDashboard.getInstance()

    // TODO: set heading PID
    // UPDATE: completed
    val turnController = PIDFController(HEADING_PID)
    lateinit var turnProfile : MotionProfile
    var turnStart = 0.0

    // TODO: set BASE_CONSTRAINTS, TRACK_WIDTH, and the other stuff
    // UPDATE: completed
    val driveConstraints = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
    val follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID)

    lateinit var lastWheelPositions : List<Double>
    var lastTimestamp = 0.0

    init {
        dashboard.telemetryTransmissionInterval = 25

        turnController.setInputBounds(0.0, 2.0 * Math.PI)

        motorsExt.forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            if (MOTOR_VELO_PID != null) setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        // TODO: Need to set localizer here to odometry system...
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = ThreeWheelOdometryLocalizer(odometryLeftExt, odometryRightExt, odometryRearExt, odometryHubExt)
    }

    val trajectoryBuilder : TrajectoryBuilder
        get() = TrajectoryBuilder(poseEstimate, driveConstraints)

    val lastError: Pose2d
        get() = when(mode) {
            Mode.FOLLOW_TRAJECTORY  -> follower.lastError
            Mode.TURN               -> Pose2d(0.0, 0.0, turnController.lastError)
            Mode.IDLE               -> Pose2d()
        }

    fun update() {
        updatePoseEstimate()

        val currentPose = poseEstimate
        val lastError = this.lastError

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)
        packet.put("heading (deg)", currentPose.heading.toDegrees())

        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)

        when (mode) {
            Mode.TURN -> {
                val t = clock.seconds() - turnStart

                val targetState = turnProfile[t]

                turnController.targetPosition = targetState.x

                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)

                setDriveSignal(
                        DriveSignal(
                                Pose2d(0.0, 0.0, targetOmega + correction),
                                Pose2d(0.0, 0.0, targetAlpha)
                        )
                )

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(currentPose))

                val trajectory = follower.trajectory

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("4CAF50")

                fieldOverlay.setStroke("3F51B5")
                fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }

        }

        dashboard.sendTelemetryPacket(packet)


    }

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory) {
        followTrajectory(trajectory)
        waitForIdle()
    }

    fun turn(angle: Double) {
        val heading = poseEstimate.heading

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(heading, 0.0, 0.0, 0.0),
                MotionState(heading + angle, 0.0, 0.0, 0.0),
                driveConstraints.maxAngVel,
                driveConstraints.maxAngAccel,
                driveConstraints.maxAngJerk
        )

        turnStart = clock.seconds()
        mode = Mode.TURN
    }

    fun turnSync(angle: Double) {
        turn(angle)
        waitForIdle()
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) update()
    }

    fun isBusy() : Boolean {
        return mode != Mode.IDLE
    }

    fun getPIDCoefficients(runMode: DcMotor.RunMode) : PIDCoefficients {
        val pidfCoefficients = frontLeftExt.getPIDFCoefficients(runMode)
        return PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        motorsExt.forEach {
            it.setPIDFCoefficients(runMode, PIDFCoefficients(coefficients.kP, coefficients.kI, coefficients.kD, 0.0))
            // TODO : "set kF to motor velocity F coefficient, look at RR quickstart DriveConstants"
        }
    }

    @NonNull
    override fun getWheelPositions() : List<Double> {
        val bulkData = motorsHubExt.bulkInputData

        bulkData ?: return listOf(0.0, 0.0, 0.0, 0.0)

        val wheelPositions = emptyList<Double>().toMutableList()

        motorsExt.forEach {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(it).toDouble()))
            // TODO: "define encoder ticks to inches method"
        }

        return wheelPositions
    }

    @NonNull
    fun getWheelVelocities() : List<Double> {
        val bulkData = motorsHubExt.bulkInputData

        bulkData ?: return listOf(0.0, 0.0, 0.0, 0.0)

        val wheelVelocities = emptyList<Double>().toMutableList()

        motorsExt.forEach {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(it).toDouble()))
            // TODO: "define encoder ticks to inches method"
        }

        return wheelVelocities
    }

    override val rawExternalHeading: Double
        get() = imuController.getHeading()

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        frontLeftExt .power =  frontLeft
        backLeftExt  .power =  rearLeft
        backRightExt .power = -rearRight
        frontRightExt.power = -frontRight
    }
}