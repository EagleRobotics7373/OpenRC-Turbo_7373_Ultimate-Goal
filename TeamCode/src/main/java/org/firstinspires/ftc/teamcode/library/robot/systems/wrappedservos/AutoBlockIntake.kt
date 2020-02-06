package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class AutoBlockIntake (
        private val pivotServo: Servo,
        private val pivot18 : Double,
        private val pivotVertical: Double,
        private val pivotMid: Double,
        private val pivotPickup: Double,
        private val grabberServo: Servo,
        private val grab18: Double,
        private val grabPickup: Double,
        private val grabUp: Double,
        private val grabMid:Double
        )
{
    fun grabBlock() { grabberServo.position = grabPickup }
    fun releaseBlock() { grabberServo.position = grabUp }
    fun grabberIn18() { grabberServo.position = grab18 }
    fun grabberMid() { grabberServo.position = grabMid }

    fun pivotDown() { pivotServo.position = pivotPickup }
    fun pivotUp() { pivotServo.position = pivotVertical }
    fun pivotIn18() { pivotServo.position = pivot18 }
    fun pivotMid() { pivotServo.position = pivotMid }
}
