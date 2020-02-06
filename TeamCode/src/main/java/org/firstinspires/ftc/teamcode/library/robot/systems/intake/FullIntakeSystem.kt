package org.firstinspires.ftc.teamcode.library.robot.systems.intake

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.IntakeBlockGrabber
import org.openftc.revextensions2.ExpansionHubMotor

class FullIntakeSystem(
        private val intakePivot : DcMotor,
        private val intakeLiftLeft: DcMotor,
        private val intakeLiftRight: DcMotor,
        private val intakeBlockGrabber: DcMotor
) {
    var intakeState: State = State.IDLE

    val intakeLift = MisumiIntakeLift(intakeLiftLeft, intakeLiftRight, 0.0)
    lateinit var intakeSensor: RevTouchSensor

    init {
        intakeLiftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    enum class State {
        LIFT_BLOCK, LOWER, PIVOT, PLACE, IDLE
    }

    fun update() {
        when (intakeState) {
            State.IDLE -> {

            }
            State.LIFT_BLOCK -> {

            }
            State.LOWER -> {

            }
            State.PIVOT -> {

            }
        }
    }

    fun beginBlockLift(numBlocks: Int) : Boolean {
        intakeState = State.LIFT_BLOCK
        return true
    }

    class MisumiIntakeLift(
            intakeLiftLeft: DcMotor,
            intakeLiftRight: DcMotor,
            startingHeight: Double
    ) {
        val spoolDiameter = 2.0
        val encoderTicksPerRevolution = 103.6

        val startingEncoderPosition = intakeLiftRight.currentPosition
        var zeroEncoderPosition = heightToEncoderTicks(startingHeight) - startingEncoderPosition

        var targetEncoderPosition = startingEncoderPosition
            private set(value) {field = value}

        fun setEncoderTarget(numBlocks: Int, extOverBlock: Boolean) {
            targetEncoderPosition = zeroEncoderPosition + heightToEncoderTicks(blocksToHeight(numBlocks))
            if (extOverBlock) targetEncoderPosition += extensionHeightOverBlock.toInt()
        }

        fun blocksToHeight(numBlocks: Int) : Double {
            return numBlocks * 4.1
        }

        val extensionHeightOverBlock : Double = 1.5

        fun heightToEncoderTicks(height: Double) : Int {
            return ((Math.PI * spoolDiameter)/(height * encoderTicksPerRevolution)).toInt()
        }
    }
}