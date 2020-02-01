package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class IntakeBlockGrabber(private val servo: Servo,
                         private val holdPos: Double,
                         private val midPos: Double,
                         private val releasePos: Double) {
    fun hold()    { servo.position = holdPos }
    fun mid()     { servo.position = midPos }
    fun release() { servo.position = releasePos }
}