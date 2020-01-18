package org.firstinspires.ftc.teamcode.library.robot.systems.intake

import com.qualcomm.robotcore.hardware.Servo

class AutoBlockIntake (
        private val pivotServo: Servo,
        private val grabberServo: Servo)
{
    private val pivotLowerBound = 0.0
    private val pivotUpperBound = 0.6

    private val grabLowerBound = 0.3
    private val grabUpperBound = 1.0

    fun pivotTo(pos: Double) {
        pivotServo.position = pivotLowerBound + pos*(pivotUpperBound-pivotLowerBound)
    }

    fun grabTo(pos: Double) {
        grabberServo.position = grabLowerBound + pos*(grabUpperBound-grabLowerBound)
    }

    fun grabBlock() = grabTo(1.0)
    fun releaseBlock() = grabTo(0.5)

    fun pivotDown() = pivotTo(0.0)
    fun pivotMid() = pivotTo(.32)
    fun pivotUp() = pivotTo(1.0)
}