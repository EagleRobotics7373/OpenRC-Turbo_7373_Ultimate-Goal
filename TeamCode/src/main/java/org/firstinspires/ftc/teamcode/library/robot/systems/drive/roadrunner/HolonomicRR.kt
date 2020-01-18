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
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.DashboardUtil
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.robotcore.OdometryRobot
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
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
             val imuController: IMUController,
             val rearOdometryContainer : OdometryModule? = null,
             val robot: OdometryRobot)

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
        localizer =
                if (useTwoWheelLocalizer)  TwoWheelOdometryLocalizer(odometryLeftExt, odometryRearExt, odometryHubExt, robot)
                else ThreeWheelOdometryLocalizer(odometryLeftExt, odometryRightExt, odometryRearExt, odometryHubExt, robot)
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

        packet.put("center module", rearOdometryContainer?.getDistanceNormalized(DistanceUnit.INCH)?.times(-1))

        val imuHeading = rawExternalHeading

        packet.put("imu heading", imuHeading)
        packet.put("imu heading (deg)", imuHeading.toDegrees())

        packet.put("heading diff (deg)", currentHeading.toDegrees() - imuHeading.toDegrees())

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

                fieldOverlay.setStroke("#3F51B5")
                fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

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
        val pidfCoefficients = frontLeftExt.getPIDFCoefficients(runMode)
        return PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        motorsExt.forEach {
            it.setPIDFCoefficients(runMode, PIDFCoefficients(coefficients.kP, coefficients.kI, coefficients.kD, kF))
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