package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.Servo

class IntakeBlockGrabber(private val servo: Servo) {
    fun hold()    { servo.position = 0.28 }
    fun mid()     { servo.position = 0.45 }
    fun release() { servo.position = 1.00 }
}