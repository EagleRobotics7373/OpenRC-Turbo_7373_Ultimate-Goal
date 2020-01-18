package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.positional.PositionalHolonomicController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR

class OdometryRobot(hardwareMap: HardwareMap) : BasicRobot(hardwareMap) {
    private   val leftOdometryModule      : DcMotor   = hwInit("leftOdometryModule")
    private   val rightOdometryModule     : DcMotor   = super.intakePivotMotor
    private   val rearOdometryModule      : DcMotor   = super.rearOdometryAsMotor

    @JvmField val imuController
          = IMUController(hardwareMap)

    @JvmField val holonomicRoadRunner
          = HolonomicRR(
                    hardwareMap,
                    frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                    leftOdometryModule, rightOdometryModule, rearOdometryModule,
                    imuController, rearOdometry, this)

    @JvmField val positionalHolonomic
          = PositionalHolonomicController(holonomic, holonomicRoadRunner, leftOdometry, rightOdometry, rearOdometry)
}