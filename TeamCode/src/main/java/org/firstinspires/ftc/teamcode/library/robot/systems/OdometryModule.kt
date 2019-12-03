package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class OdometryModule(private val encoder: DcMotor) {

    init {
        encoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private var currentPositionOffset = 0

    fun resetHWCounter() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun resetSWCounter() { currentPositionOffset = encoder.currentPosition }

    fun getDistanceRaw() = (encoder.currentPosition - currentPositionOffset)

    fun getDistanceNormalized(unit: DistanceUnit = DistanceUnit.INCH) = unit.fromMm(((encoder.currentPosition - currentPositionOffset).toDouble()/1440) * Math.PI * 38)
}