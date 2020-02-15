package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.functions.reverseIf

class FoundationGrabbers(
        private val leftServo: Servo,
        private val leftLock: Double,
        private val leftUnlock: Double,
        private val leftMid: Double,
        private val rightServo: Servo,
        private val rightLock: Double,
        private val rightUnlock: Double,
        private val rightMid: Double) {

//    val leftPos = {pos:Double -> if (leftInv) leftMax - (leftMax - leftMin)*pos else leftMin + (leftMax-leftMin)*pos }
//    val rightPos= {pos:Double -> if (rightInv) rightMax - (rightMax - rightMin)*pos else rightMin + (rightMax-rightMin*pos) }

    fun unlock() {
        leftServo.position = leftUnlock
        rightServo.position = rightUnlock
    }

    fun lock() {
        leftServo.position = leftLock
        rightServo.position = rightLock
    }

    fun mid() {
        leftServo.position = leftMid
        rightServo.position = rightMid
    }

//    fun setPosition(pos: Double) {
//        leftServo.position = leftPos(pos)
//        rightServo.position = rightPos(pos)
//    }
}