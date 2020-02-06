package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class CapstonePlacer(private val servo: Servo,
                     private val pos18: Double,
                     private val posInside: Double,
                     private val posDeploy: Double) {
    fun moveIn18()    { servo.position = pos18 }
    fun moveInside()     { servo.position = posInside }
    fun moveDeploy() { servo.position = posDeploy }
}