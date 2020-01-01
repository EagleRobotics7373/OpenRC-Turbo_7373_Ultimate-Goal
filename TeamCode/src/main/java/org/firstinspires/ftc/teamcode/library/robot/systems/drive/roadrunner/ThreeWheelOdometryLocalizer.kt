package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor

class ThreeWheelOdometryLocalizer(
        private val leftModule : ExpansionHubMotor,
        private val rightModule: ExpansionHubMotor,
        private val rearModule : ExpansionHubMotor,
        private val expansionHub: ExpansionHubEx
)
    : ThreeTrackingWheelLocalizer(
        listOf(
                Pose2d(-0.39, -7.32, 0.0),
                Pose2d(-0.39, 7.32, 0.0),
                Pose2d(-6.93, 0.0, Math.PI / 2)
        )
    )
{
    val modulesExt = listOf(leftModule, rightModule, rearModule)

    val TICKS_PER_REVOLUTION = 8192
    val WHEEL_DIAMETER_mm = DistanceUnit.INCH.fromMm(38.0)

    init {
        modulesExt.forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    override fun getWheelPositions(): List<Double> {
        val bulkData = expansionHub.bulkInputData

        val wheelPositions = emptyList<Double>().toMutableList()

        modulesExt.forEach {
            wheelPositions.add(-(bulkData.getMotorCurrentPosition(it).toDouble() / TICKS_PER_REVOLUTION) * WHEEL_DIAMETER_mm * Math.PI)
        }

        return wheelPositions
    }


}