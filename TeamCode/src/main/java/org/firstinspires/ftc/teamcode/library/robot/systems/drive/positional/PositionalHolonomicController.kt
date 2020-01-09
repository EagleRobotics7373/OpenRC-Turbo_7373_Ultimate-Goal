package org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import kotlin.math.absoluteValue

class PositionalHolonomicController(
        val holonomic: Holonomic,
        val holonomicRR: HolonomicRR,
        val leftModule: OdometryModule,
        val rightModule: OdometryModule,
        val rearModule: OdometryModule) {

    var mode = Mode.INACTIVE
    val dashboard: FtcDashboard = FtcDashboard.getInstance()
    var globalHeading = holonomicRR.externalHeading

    private lateinit var positionTarget : PositionConstruct

    enum class Mode {
        INACTIVE, TARGETING
    }

    fun update() {
        holonomicRR.update()

        if (mode == Mode.TARGETING) {


            val output = positionTarget.recieveOutput(getPoseEstimate())
            holonomic.runWithoutEncoder(-output.y, output.x, output.heading)

            if ((if (positionTarget.ignoreX) true else positionTarget.lastError.x.absoluteValue < 0.5)
                    && (if (positionTarget.ignoreY) true else positionTarget.lastError.y.absoluteValue < 0.5)
                    && (if (positionTarget.ignoreHeading) true else positionTarget.lastError.heading.absoluteValue < 0.1)) {
                mode = Mode.INACTIVE
                holonomic.stop()
                holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)
            }

            val packet = TelemetryPacket()
            packet.put("x error", positionTarget.lastError.x)
            packet.put("y error", positionTarget.lastError.y)
            packet.put("heading error", positionTarget.lastError.heading)
            packet.put("heading error (deg)", positionTarget.lastError.heading)

            packet.put("x output (rcv)", output.x)
            packet.put("y output (rcv)", output.y)
            packet.put("heading output (rcv)", output.heading)

            packet.put("x error sum", positionTarget.errorSum.x)
            packet.put("y error sum", positionTarget.errorSum.y)
            packet.put("heading error sum", positionTarget.errorSum.heading)

            dashboard.sendTelemetryPacket(packet)

        }
    }

    private fun runPosition(target: PositionConstruct) {
        mode = Mode.TARGETING
        positionTarget = target
    }

    fun runPositionSync(target: PositionConstruct) {
        runPosition(target)
        while (!Thread.currentThread().isInterrupted && mode!=Mode.INACTIVE) update()
    }

    private fun getPoseEstimate() : Pose2d {
        return Pose2d(
                (leftModule.getDistanceNormalized(DistanceUnit.INCH) + rightModule.getDistanceNormalized(DistanceUnit.INCH)) / 2,
                -rearModule.getDistanceNormalized(DistanceUnit.INCH),
                holonomicRR.externalHeading
        )
    }

    fun resetOdometry() {
        leftModule.resetSWCounter()
        rightModule.resetSWCounter()
        rearModule.resetSWCounter()
    }

    @JvmOverloads fun positionConstructor(resetHeadingTarget: Boolean = false): PositionConstructor {
        resetOdometry()
        val currentPose = getPoseEstimate()
        return PositionConstructor(currentPose.x, currentPose.y, holonomicRR.externalHeading, resetHeadingTarget)
    }
}