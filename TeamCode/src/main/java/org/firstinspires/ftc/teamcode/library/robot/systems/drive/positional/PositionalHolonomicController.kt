package org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import kotlin.math.absoluteValue

class PositionalHolonomicController(val holonomic: Holonomic, val holonomicRR: HolonomicRR) {

    var mode = Mode.INACTIVE
    val dashboard: FtcDashboard = FtcDashboard.getInstance()

    private lateinit var positionTarget : PositionConstruct

    enum class Mode {
        INACTIVE, TARGETING
    }

    fun update() {
        holonomicRR.update()

        if (mode == Mode.TARGETING) {
            val output = positionTarget.recieveOutput()
            holonomic.runWithoutEncoder(-output.y, output.x, output.heading)

            if (positionTarget.lastError.x.absoluteValue < 0.5
                    && positionTarget.lastError.y.absoluteValue < 0.5
                    && positionTarget.lastError.heading.absoluteValue < 1) {
                mode = Mode.INACTIVE
                holonomic.stop()
                holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)
            }

            val packet = TelemetryPacket()
            packet.put("x error", positionTarget.lastError.x)
            packet.put("y error", positionTarget.lastError.y)
            packet.put("heading error", positionTarget.lastError.heading)

            packet.put("x output (rcv)", output.x)
            packet.put("y output (rcv)", output.y)
            packet.put("heading output (rcv)", output.heading)

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

    fun positionConstructor(): PositionConstructor {
        val currentPose = holonomicRR.poseEstimate
        return PositionConstructor(holonomicRR, currentPose.x, currentPose.y, currentPose.heading)
    }
}