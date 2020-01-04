package org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import kotlin.math.absoluteValue

class PositionConstructor(
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

    private var maxPowers  = Pose2d(1.0, 1.0, 1.0)

    private var errorSumRestrictOutOf = Pose2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)

    private var ignoreX = false
    private var ignoreY = false
    private var ignoreHeading = false

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

    fun setIgnoreAxes(x: Boolean = ignoreX, y: Boolean = ignoreY, heading: Boolean = ignoreHeading) : PositionConstructor{
        ignoreX = x
        ignoreY = y
        ignoreHeading = heading
        return this
    }

    fun setDriveProfile(profile: DriveProfile): PositionConstructor {
        xAxisPID = profile.xAxisPID
        yAxisPID = profile.yAxisPID
        headingPID = profile.headingPID

        maxPowers = profile.maxOutputs

        errorSumRestrictOutOf = profile.integralRestriction

        return this
    }

    @JvmOverloads fun setSumRestrict(x: Double = errorSumRestrictOutOf.x, y: Double = errorSumRestrictOutOf.y, z: Double = errorSumRestrictOutOf.heading) : PositionConstructor {
        errorSumRestrictOutOf = Pose2d(x, y, z)
        return this
    }

    fun setMaxPowers(x: Double = maxPowers.x, y: Double = maxPowers.y, z: Double = maxPowers.heading): PositionConstructor {
        maxPowers = Pose2d(x.absoluteValue, y.absoluteValue, z.absoluteValue)
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
//
//    fun vectorRelative(distXIn: Double, distYIn: Double): PositionConstructor {
//        return this
//    }

    @JvmOverloads fun vectorRelative(distXIn: Double, distYIn: Double, headingDegrees: Double = 0.0) : PositionConstructor {
        xTarget += distXIn
        yTarget += distYIn
        headingTarget += headingDegrees.toRadians()
        return this
    }

    fun build() : PositionConstruct {
        return PositionConstruct(
                Pose2d(xTarget, yTarget, headingTarget),
                xAxisPID, yAxisPID, headingPID,
                maxPowers,
                errorSumRestrictOutOf,
                ignoreX, ignoreY, ignoreHeading
        )
    }

    enum class DriveProfile(
            val xAxisPID            : PIDCoefficients,
            val yAxisPID            : PIDCoefficients,
            val headingPID          : PIDCoefficients,
            val maxOutputs          : Pose2d,
            val integralRestriction : Pose2d
    ) {
//        SHORT_DIST(
//                PIDCoefficients(0.03, 1.5, 0.0),
//                PIDCoefficients(0.3, 0.01, 0.0),
//                PIDCoefficients(1.5, 0.0, 0.0),
//                Pose2d(0.2, 0.2, 0.2),
//                Pose2d(2.0, 2.0, Math.PI / 2)
//        ),
        STRAFE_X(
                PIDCoefficients(0.1, 0.055, 0.0),
                PIDCoefficients(0.03, 0.13, 0.0),
                PIDCoefficients(1.5, 0.0, 0.0),
                Pose2d(0.6, 0.6, 0.6),
                Pose2d(5.0, 5.0, Math.PI / 2)
        ),
        STRAFE_Y(
                PIDCoefficients(0.17, 0.02, 0.0),
                PIDCoefficients(0.025, 0.13, 0.0),
                PIDCoefficients(1.5, 0.0, 0.0),
                Pose2d(0.6, 0.6, 0.6),
                Pose2d(5.0, 5.0, Math.PI / 2)
        )
    }

}

class PositionConstruct(
        val targets: Pose2d,
        val xAxisPID: PIDCoefficients,
        val yAxisPID: PIDCoefficients,
        val headingPID: PIDCoefficients,
        val maxOutputs: Pose2d,
        val errorSumRestrictOutOf: Pose2d,
        val ignoreX: Boolean,
        val ignoreY: Boolean,
        val ignoreHeading: Boolean) {

    enum class Mode {
        INACTIVE, RUNNING
    }

    var mode = Mode.INACTIVE

    lateinit var lastError: Pose2d
    var errorSum  = Pose2d(0.0, 0.0, 0.0)
    var lastUpdate : Long = 0

    fun recieveOutput(poseEstimate: Pose2d) : Pose2d {
        if (mode == Mode.INACTIVE) {
            mode = Mode.RUNNING
            lastError = getError(poseEstimate)
            lastUpdate = System.currentTimeMillis() - 1
        }

        val error = getError(poseEstimate)
        val currentTime = System.currentTimeMillis()
        val timeDiff = currentTime - lastUpdate

        val errorDelta = Pose2d(
                ((error.x-lastError.x)/timeDiff),
                ((error.y-lastError.y)/timeDiff),
                ((error.heading-lastError.heading)/timeDiff)
        )

        errorSum = Pose2d(
                errorSum.x + if (error.x.absoluteValue < errorSumRestrictOutOf.x) error.x/timeDiff else 0.0,
                errorSum.y + if (error.y.absoluteValue < errorSumRestrictOutOf.y) error.y/timeDiff else 0.0,
                errorSum.heading + if (error.heading.absoluteValue < errorSumRestrictOutOf.heading) error.heading/timeDiff else 0.0
        )

        val output = Pose2d(
                (xAxisPID.kP * error.x               + xAxisPID.kI * errorSum.x           + xAxisPID.kD * errorDelta.x).coerceIn(-maxOutputs.x, maxOutputs.x),
                (yAxisPID.kP * error.y               + yAxisPID.kI * errorSum.y           + yAxisPID.kD * errorDelta.y).coerceIn(-maxOutputs.y, maxOutputs.y),
                (headingPID.kP * error.heading  + headingPID.kI * errorSum.heading   + headingPID.kD * errorDelta.heading).coerceIn(-maxOutputs.heading, maxOutputs.heading).times(-1)
        )

        lastError = error
        lastUpdate = currentTime

        return output
    }

    private fun getError(poseEstimate: Pose2d): Pose2d {
        var currentHeading = poseEstimate.heading

        if (currentHeading > Math.PI) currentHeading -= (Math.PI * 2)

        var targetHeading = targets.heading
        if (targetHeading > Math.PI) targetHeading -= (Math.PI * 2)

        return Pose2d(
                targets.x - poseEstimate.x,
                targets.y - poseEstimate.y,
                targetHeading - currentHeading )
    }
}