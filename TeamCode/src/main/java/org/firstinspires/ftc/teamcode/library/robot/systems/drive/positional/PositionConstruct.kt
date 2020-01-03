package org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR

class PositionConstructor(
        val drive: HolonomicRR,
        val xStarting: Double,
        val yStarting: Double,
        var headingStarting: Double)
{
    private var xTarget: Double = xStarting
    private var yTarget: Double = yStarting
    private var headingTarget: Double = headingStarting

    private var xAxisPID   = PIDCoefficients(1.0, 0.0, 0.0)
    private var yAxisPID   = PIDCoefficients(1.0, 0.0, 0.0)
    private var headingPID = PIDCoefficients(1.0, 0.0, 0.0)

    fun setXAxisPID(pid : PIDCoefficients): PositionConstructor  {
        xAxisPID = pid
        return this
    }

    fun setYAxisPID(pid : PIDCoefficients): PositionConstructor  {
        yAxisPID = pid
        return this
    }

    fun setHeadingPID(pid : PIDCoefficients): PositionConstructor  {
        headingPID = pid
        return this
    }

    fun forward(distIn: Double): PositionConstructor {
        xTarget += distIn
        return this
    }

    fun backward(distIn: Double): PositionConstructor {
        xTarget -= distIn
        return this
    }

    fun strafeLeft(distIn: Double) : PositionConstructor {
        yTarget += distIn
        return this
    }

    fun strafeRight(distIn: Double) : PositionConstructor {
        yTarget -= distIn
        return this
    }

    fun rotate(degrees: Double) : PositionConstructor {
        headingTarget += degrees
        while (headingTarget >= 360) headingTarget -= 360
        while (headingTarget < 0) headingTarget += 360
        return this
    }

    fun vectorTo(posX: Double, posY: Double): PositionConstructor {
        xTarget = posX
        yTarget = posY
        return this
    }

    fun vectorRelative(distXIn: Double, distYIn: Double): PositionConstructor {
        xTarget += distXIn
        yTarget += distYIn
        return this
    }

    fun build() : PositionConstruct {
        return PositionConstruct(
                drive,
                Pose2d(xTarget, yTarget, headingTarget),
                xAxisPID,
                yAxisPID, headingPID
        )
    }

}

class PositionConstruct(
        val drive: HolonomicRR,
        val targets: Pose2d,
        val xAxisPID: PIDCoefficients,
        val yAxisPID: PIDCoefficients,
        val headingPID: PIDCoefficients) {

    enum class Mode {
        INACTIVE, RUNNING
    }

    var mode = Mode.INACTIVE

    lateinit var lastError: Pose2d
    var errorSum  = Pose2d(0.0, 0.0, 0.0)
    var lastUpdate : Long = 0

    fun recieveOutput() : Pose2d {
        if (mode == Mode.INACTIVE) {
            mode = Mode.RUNNING
            lastError = getError()
            lastUpdate = System.currentTimeMillis()
        }

        val error = getError()
        val currentTime = System.currentTimeMillis()
        val timeDiff = currentTime - lastUpdate

        val errorDelta = Pose2d(
                ((error.x-lastError.x)/timeDiff),
                ((error.y-lastError.y)/timeDiff),
                ((error.heading-lastError.heading)/timeDiff)
        )

        errorSum = Pose2d(
                errorSum.x + errorDelta.x,
                errorSum.y + errorDelta.y,
                errorSum.heading + errorDelta.heading
        )

        val output = Pose2d(
                xAxisPID.kP * error.x               + xAxisPID.kI * errorSum.x           + xAxisPID.kD * errorDelta.x,
                yAxisPID.kP * error.y               + yAxisPID.kI * errorSum.y           + yAxisPID.kD * errorDelta.y,
                headingPID.kP * error.heading  + headingPID.kI * errorSum.heading   + headingPID.kD * errorDelta.heading
        )

        return output
    }

    private fun getError(): Pose2d {
        val currentPose = drive.poseEstimate
        return Pose2d(
                targets.x - currentPose.x,
                targets.y - currentPose.y,
                targets.heading - currentPose.heading )
    }
}