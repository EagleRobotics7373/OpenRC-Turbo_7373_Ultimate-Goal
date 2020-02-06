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
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.DashboardUtil
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsNew.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsOld.globalPoseEstimate


class HolonomicRR

constructor (hardwareMap: HardwareMap,
             private val imuController: IMUController,
             private val frontLeftExt: ExpansionHubMotor,
             private val backLeftExt:  ExpansionHubMotor,
             private val backRightExt: ExpansionHubMotor,
             private val frontRightExt:ExpansionHubMotor,
             localizer: Localizer)

    : MecanumDrive(kV, kA, kStatic, TRACK_WIDTH) // TODO: Define these variables
{
    constructor(_hardwareMap: HardwareMap,
                _imuController: IMUController,
                _frontLeftMotor: DcMotor,
                _backLeftMotor: DcMotor,
                _backRightMotor: DcMotor,
                _frontRightMotor: DcMotor,
                _localizer: Localizer) : this(
            hardwareMap =  _hardwareMap,
            imuController = _imuController,
            frontLeftExt = (_frontLeftMotor.takeIf { it is ExpansionHubMotor } ?: _hardwareMap.get(ExpansionHubMotor::class.java, _hardwareMap.getNamesOf(_frontLeftMotor).first())) as ExpansionHubMotor,
            backLeftExt =  (_backLeftMotor.takeIf { it is ExpansionHubMotor } ?: _hardwareMap.get(ExpansionHubMotor::class.java, _hardwareMap.getNamesOf(_backLeftMotor).first())) as ExpansionHubMotor,
            backRightExt =  (_backRightMotor.takeIf { it is ExpansionHubMotor } ?: _hardwareMap.get(ExpansionHubMotor::class.java, _hardwareMap.getNamesOf(_backRightMotor).first())) as ExpansionHubMotor,
            frontRightExt = (_frontRightMotor.takeIf { it is ExpansionHubMotor } ?: _hardwareMap.get(ExpansionHubMotor::class.java, _hardwareMap.getNamesOf(_frontRightMotor).first())) as ExpansionHubMotor,
            localizer = _localizer
    )

    val motorsExt = listOf(frontLeftExt, backLeftExt, backRightExt, frontRightExt)
    val hubA = hardwareMap.get(ExpansionHubEx::class.java,"Expansion Hub A")
    val hubB = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub B")

    enum class Mode { IDLE, TURN, FOLLOW_TRAJECTORY }
    var mode = Mode.IDLE

    val constraints = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)

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
    val follower = HolonomicPIDVAFollower(TRANSLATIONAL_X_PID, TRANSLATIONAL_Y_PID, HEADING_PID)

    private lateinit var lastWheelPositions : List<Double>
    private var lastTimestamp = 0.0
    private var lastReadTime : Long = 0

    private var currentDriveSignal = DriveSignal()
//    private val driveSignalUpdateRunnable = {
//        try {
//            while (isBusy()) {
//                setDriveSignal(currentDriveSignal)
//                print("CURRENT SIGNAL = $currentDriveSignal")
//            }
//        } catch (e: InterruptedException) {
//            e.printStackTrace()
//        }
//    }
//    private var driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)


    init {
        dashboard.telemetryTransmissionInterval = 25
        turnController.setInputBounds(0.0, 2.0 * Math.PI)

        if (globalPoseEstimate != null) poseEstimate = globalPoseEstimate

//        Thread { while (!Thread.interrupted()) updatePoseEstimate() }.start()

        motorsExt.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            if (RUN_USING_ENCODER) {
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
                if (MOTOR_VELO_PID != null) setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
            }
        }

        // TODO: Need to set localizer here to odometry system...
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        super.localizer = localizer
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
        val beforePoseUpdate = System.currentTimeMillis()
        updatePoseEstimate()
        val afterPoseUpdate = System.currentTimeMillis()

        val currentPose = poseEstimate

        val afterGetPose = System.currentTimeMillis()
        var beforeGetDriveSignal = Long.MIN_VALUE
        var afterGetDriveSignal = Long.MIN_VALUE
        val lastError = this.lastError

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        val currentHeading = currentPose.heading

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentHeading)
        packet.put("heading (deg)", currentHeading.toDegrees())

        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)

        val imuHeading = rawExternalHeading

        packet.put("imu heading", imuHeading)
        packet.put("imu heading (deg)", imuHeading.toDegrees())

        packet.put("heading diff (deg)", currentHeading.toDegrees() - imuHeading.toDegrees())

        fieldOverlay.setStroke("#3F51B5")
        fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

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
                val toSet = follower.update(currentPose)
                beforeGetDriveSignal = System.currentTimeMillis()
                setDriveSignal(toSet)
                afterGetDriveSignal = System.currentTimeMillis()

                val trajectory = follower.trajectory

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("#4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)

                fieldOverlay.setStroke("#F44336")
                val time = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[time])

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }

        }

        val endReadTime = System.currentTimeMillis()
        println()
        print("%% HolonomicRR_updates\tREAD GAP = ${beforePoseUpdate - lastReadTime}")
        lastReadTime = endReadTime
        print("\tAFTER POSE UPDATE = ${afterPoseUpdate - beforePoseUpdate}")
        print("\tAFTER GET POSE = ${afterGetPose - beforePoseUpdate}")
        print("\tAFTER UPDATE FOLLOWER = ${beforeGetDriveSignal - beforePoseUpdate}")
        print("\tAFTER GET DRIVE SIGNAL = ${afterGetDriveSignal - beforePoseUpdate}")
        print("\tUPDATE TIME = ${endReadTime - beforePoseUpdate}\t")
        print("\t %% HolonomicRR_pose\t X=${poseEstimate.x}\t Y=${poseEstimate.y}\t HEADING=${currentHeading}\t IMU_HEADING=${imuHeading}\t   XERROR=${lastError.x}\t YERROR=${lastError.y}\t HEADINGERROR=${lastError.heading}")
        dashboard.sendTelemetryPacket(packet)

        globalPoseEstimate = poseEstimate
    }

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory) {
        followTrajectory(trajectory)
        waitForIdle()
    }

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints)
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
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
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
        val pidfCoefficients = (frontLeftExt as DcMotorEx).getPIDFCoefficients(runMode)
        return PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        motorsExt.forEach {
            (it as DcMotorEx).setPIDFCoefficients(runMode, PIDFCoefficients(coefficients.kP, coefficients.kI, coefficients.kD, kF))
            // TODO : "set kF to motor velocity F coefficient, look at RR quickstart DriveConstantsNew"
        }
    }

    @NonNull
    override fun getWheelPositions() : List<Double> {
        val bulkDataA = hubA.bulkInputData
        val bulkDataB = hubB.bulkInputData

        bulkDataA ?: return listOf(0.0, 0.0, 0.0, 0.0)

        val wheelPositions = emptyList<Double>().toMutableList()

        wheelPositions.add(encoderTicksToInches(bulkDataA.getMotorCurrentPosition(frontLeftExt).toDouble()))
        wheelPositions.add(encoderTicksToInches(bulkDataA.getMotorCurrentPosition(backLeftExt).toDouble()))
        wheelPositions.add(encoderTicksToInches(bulkDataB.getMotorCurrentPosition(backRightExt).toDouble()))
        wheelPositions.add(encoderTicksToInches(bulkDataB.getMotorCurrentPosition(frontRightExt).toDouble()))

        return wheelPositions
    }

    @NonNull
    fun getWheelVelocities() : List<Double> {
        val bulkDataA = hubA.bulkInputData
        val bulkDataB = hubB.bulkInputData

        bulkDataA ?: return listOf(0.0, 0.0, 0.0, 0.0)

        val wheelVelocities = emptyList<Double>().toMutableList()

        wheelVelocities.add(encoderTicksToInches(bulkDataA.getMotorVelocity(frontLeftExt).toDouble()))
        wheelVelocities.add(encoderTicksToInches(bulkDataA.getMotorVelocity(backLeftExt).toDouble()))
        wheelVelocities.add(encoderTicksToInches(bulkDataB.getMotorVelocity(backRightExt).toDouble()))
        wheelVelocities.add(encoderTicksToInches(bulkDataB.getMotorVelocity(frontRightExt).toDouble()))

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