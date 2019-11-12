package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot
import kotlin.math.absoluteValue

@TeleOp(name="TeleOp RD", group="basic")
open class TeleOpRD : OpMode() {
    lateinit var robot : BasicRobot

    protected lateinit var watch_gamepad1_buttonY : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp : ToggleButtonWatcher
    protected lateinit var watch_gamepad2_rightStickButton : ToggleButtonWatcher

    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.UNITY)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var reverse = false
    var speed = 3

    override fun init() {
        robot = BasicRobot(hardwareMap)
        watch_gamepad1_buttonY = ToggleButtonWatcher {gamepad1.y}
        watch_gamepad1_dpadDown = ToggleButtonWatcher {gamepad1.dpad_down}
        watch_gamepad1_dpadUp = ToggleButtonWatcher {gamepad1.dpad_up}
        watch_gamepad2_rightStickButton = ToggleButtonWatcher { gamepad2.right_stick_button }
    }
// speed control, down = 50%, down = 25% (done with 1/3s)
    // triggers for wheels, right = block goes up (done)
    //right bumper close
    //left bumper open
    //right stick push for barely out

    override fun loop() {
        controlDrivetrain()
        controlFoundationGrabbers()
        controlIntakeMechanism()
        controlTelemetry()
        controlMusic()
    }

    override fun stop() {
        super.stop()
        musicPlayer.stop()
    }

    protected open fun controlDrivetrain() {
        val x = gamepad1.left_stick_x .toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed).reverseIf(reverse)
        val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed).reverseIf(reverse)
        val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed)


//        robot.holonomic.runWithoutEncoderVectored(x, y, z, 0.0)
        robot.holonomic.runWithoutEncoder(x, y, z)
        if (gamepad1.a) reverse = false
        else if (gamepad1.b) reverse = true

        if (watch_gamepad1_buttonY.call()) reverse = !reverse
        // do y for toggle
        if (watch_gamepad1_dpadDown.call() and (speed > 1)) speed--
        if (watch_gamepad1_dpadUp.call() and (speed < 3)) speed++


    }

    private fun controlFoundationGrabbers() {
        if (gamepad2.dpad_down) robot.foundationGrabbers.lock()
        else if (gamepad2.dpad_up) robot.foundationGrabbers.unlock()
    }

    private var intakeP = 4.0
    private var intakeI = 0.000
    private var intakeD = 0.20
    private val voltageTarget = 1.257
    private var lastRuntime = -1.0
    private var d = 0.0
    private var lastError = 0.0
    private var errorSum = 0.0
    private fun controlIntakeMechanism() {
        // control intake arm up/down
        val input =
                if (gamepad2.left_stick_button) {
                    //use pid loop
                    val error = voltageTarget-robot.intakePivotPotentiometer.voltage
                    if (lastRuntime==-1.0) lastRuntime = runtime
                    errorSum += error*runtime-lastRuntime

                    d = error-lastError
                    lastRuntime = runtime
                    lastError = error

                    -(intakeP * error + intakeI * errorSum + intakeD * d)
                }
                else gamepad2.left_stick_y.toDouble()

        robot.intakePivotMotor.power =
                when {
                    (gamepad2.right_bumper)-> if (input > 0.0) input+0.02 else 0.02 - -input*0.04
                    input > 0.0 -> input+0.12
                    else -> 0.12 - -input*0.14
                }/*.rangeBuffer(-0.10, 0.10, 0.0)*/

        // control block grabbing
        if (gamepad2.right_stick_y.toDouble().absoluteValue > 0.10) robot.intakeBlockManipulator.power = -gamepad2.right_stick_y.toDouble()
        else if (gamepad1.right_trigger > 0.03) robot.intakeBlockManipulator.power = gamepad1.right_trigger.toDouble()
        else if (gamepad1.left_trigger > 0.03) robot.intakeBlockManipulator.power = -gamepad1.left_trigger.toDouble()
        else robot.intakeBlockManipulator.power = 0.0

        // control grabbing servo
        if (gamepad2.x or gamepad1.right_bumper) robot.intakeBlockGrabber.hold()
        else if (gamepad2.y or gamepad1.left_bumper) robot.intakeBlockGrabber.release()
        else if (gamepad2.a or gamepad1.right_stick_button) robot.intakeBlockGrabber.mid()

    }

    private fun controlMusic() {
        if (watch_gamepad2_rightStickButton.call()) {
            if (musicPlayer.isPlaying()) musicPlayer.pause()
            else musicPlayer.play()
        }
    }

    private fun controlTelemetry() {
        telemetry.addData("Intake manipulator power", robot.intakeBlockManipulator.power)
        telemetry.addData("Intake pivot power", robot.intakePivotMotor.power)
//        telemetry.addData("Potentiometer voltage", robot.intakePivotPotentiometer.voltage)
//        telemetry.addData("Potentiometer max v", robot.intakePivotPotentiometer.maxVoltage)
//        telemetry.addData("Front dist gD()", robot.frontDistanceSensor.getDistance(DistanceUnit.CM))
//        telemetry.addData("Front dist cU()", robot.frontDistanceSensor.cmUltrasonic())
        telemetry.addData("Speed", speed)
        telemetry.addData("Reverse", reverse)
//        telemetry.addData("Hue in range?", robot.intakeBlockCSensor.rhue in 0.32..0.41)
//        telemetry.addData("hue", robot.intakeBlockCSensor.rhue)
//        telemetry.addData("DS distance", robot.intakeBlockDSensor.getDistance(DistanceUnit.MM))
        telemetry.update()
    }
    fun setCoeffs(p: Double, i: Double, d: Double) {
        intakeP = p
        intakeI = i
        intakeD = d
    }

    protected class ToggleButtonWatcher(private val getMethod: ()->Boolean) {
        private var lastState : Boolean = getMethod()
        fun call(): Boolean {
            if (getMethod()) {
                if (!lastState) {
                    lastState = true
                    return true
                }
            } else lastState = false
            return false
        }
    }
}



