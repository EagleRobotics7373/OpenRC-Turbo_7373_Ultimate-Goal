package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.Servo

class FoundationGrabbers(
        private val leftServo: Servo,
        private val rightServo: Servo) {

    val leftPos = {pos:Double -> 0.11764 + 0.88236 * pos}
    val rightPos= {pos:Double -> 0.90588 - 0.90588 * pos}

    fun unlock() {
        setPosition(0.0)
    }

    fun lock() {
        setPosition(0.95)
    }

    fun setPosition(pos: Double) {
        leftServo.position = leftPos(pos)
        rightServo.position= rightPos(pos)
    }
}