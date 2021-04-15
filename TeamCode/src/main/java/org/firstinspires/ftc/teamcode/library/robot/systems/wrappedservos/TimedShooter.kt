package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants

class TimedShooter(private val servo: Servo) {

    var queue: Int = 0
        private set

    var lastChange = System.currentTimeMillis()
        private set

    var position: Position = TimedShooter.Position.STORAGE
        private set(newValue) { field = newValue; lastChange = System.currentTimeMillis() }

    val nextPosition: Position
        get() = if (position == Position.STORAGE) Position.HIT else Position.STORAGE

    val positionDuration: Long
        get() = when(position) {
            Position.HIT -> ExtZoomBotConstants.TELEOP_SHOOT_WAIT_HIT.toLong()
            Position.STORAGE -> ExtZoomBotConstants.TELEOP_SHOOT_WAIT_STORED.toLong()
        }

    enum class Position {
        HIT, STORAGE
    }

    fun hit(rings: Int = 1) {
        queue += kotlin.math.min(rings, 3)
    }

    fun resetQueue() {
        queue = 0
    }

    fun update(overrideServoPosition: Double? = null) {
        if (overrideServoPosition == null) {
            if (queue > 0 && System.currentTimeMillis() >= lastChange + positionDuration) {
                position = nextPosition
                if (position == Position.STORAGE) queue--
            }

            servo.position = when(position) {
                Position.HIT -> ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
                Position.STORAGE -> ExtZoomBotConstants.RING_LOAD_SERVO_BACK
            }
        } else {
            servo.position = overrideServoPosition
        }
    }
}