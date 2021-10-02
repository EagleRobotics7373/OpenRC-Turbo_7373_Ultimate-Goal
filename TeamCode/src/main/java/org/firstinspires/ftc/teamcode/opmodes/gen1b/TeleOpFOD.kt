package org.firstinspires.ftc.teamcode.opmodes.gen1b

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.rangeBuffer

/**
 * Driver-operated opmode for field-oriented drive, as an extension of base TeleOp functions
 */
@TeleOp(name="TeleOp FOD Gen2", group="Gen2 Basic")
class TeleOpFOD : TeleOpRD() {
    // when this is true, field-oriented drive is enabled
    var fod = false

    // baseline value for IMU heading value
    var zeroAngle = 0.0

    // imu controller; uses lateinit keyword to signify instantiation after opmode init

    override fun init() {
        super.init()
    }

    /**
     * override function that runs instead of TeleOpRD.controlDrivetrain()
     */
    override fun controlDrivetrain() {
        // enable field-oriented drive if 'x' button is pressed
        if (canControlIntakeOrientation && gamepad1.x) {
            fod = true
        }
        // disable field-oriented drive if 'a' or 'b' buttons are pressed
        else if ((gamepad1.left_bumper && gamepad1.right_bumper && !gamepad1.start) && (gamepad1.a or gamepad1.b)) fod = false


        if (fod)
        {
            // reset IMU baseline if left stick button is pressed
            if (gamepad1.left_stick_button) zeroAngle = robot.imuControllerC.getHeading()

            // Set x, y, and z inputs:
            //    Use Double.rangeBuffer to set motor power dead-band
            //    Use Double.times() for variable speed. (This functional method works the same as doing x * y in Java)
            val x = gamepad1.left_stick_x.toDouble().rangeBuffer(-0.15, 0.15, 0.0).times(0.33*speed)
            val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.15, 0.15, 0.0).times(0.33*speed)
            val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.15, 0.15, 0.0).times(0.33*speed)

            // Set drivetrain values, including an offset angle for FOD
            robot.holonomic.runWithoutEncoderVectored(x, y, z, zeroAngle - robot.imuControllerC.getHeading())
        }
        // if FOD is not enabled, invoke superclass relative driving method instead
        else super.controlDrivetrain()

        // speed control using button watchers
        if (watch_gamepad1_dpadDown.invoke() and (speed > 1)) speed--
        if (watch_gamepad1_dpadUp.invoke() and (speed < 3)) speed++

        // basic telemetry
        telemetry.addData("fod", fod)
        telemetry.addData("zero angle", zeroAngle)
    }

}