package org.firstinspires.ftc.teamcode.library.robot.robotcore.legacyconfig

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.TwoWheelOdometryLocalizer

@Deprecated("replaced by ExtPushBot")
open class PushBot(protected val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor       : DcMotorEx             = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor        : DcMotorEx             = hwInit("backLeftMotor")
     @JvmField val frontRightMotor      : DcMotorEx             = hwInit("frontRightMotor")
     @JvmField val backRightMotor       : DcMotorEx             = hwInit("backRightMotor")

     @JvmField val odometryLeft         : DcMotorEx             = hwInit("odometryLeft")
     @JvmField val odometryRight        : DcMotorEx             = hwInit("odometryRight")
     @JvmField val odometryRear         : DcMotorEx             = hwInit("odometryRear")

    // Expansion Hub Variables
     @JvmField val expansionhubs           : List<LynxModule>      = hardwareMap.getAll(LynxModule::class.java).apply { forEach {it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO} }

    // IMU Variables
     @JvmField val imuControllerA          : IMUController = IMUController(hardwareMap = hardwareMap, id = 'A')

    // Robot Systems Variables
     @JvmField val holonomic               : Holonomic = Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)

     @JvmField val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerA,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 TwoWheelOdometryLocalizer(odometryLeft, odometryRear, imuControllerA))

    protected inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
