package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.FoundationGrabbers
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.CapstonePlacer
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.IntakeBlockGrabber

open class ExtMisumiRobot(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtMisumiRobot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsTunedMisumi::class.java,
                OdometryConstants::class.java
        )
    }

    // Drivetrain Variables
            // @JvmField val frontLeftMotor  : DcMotorEx FROM BaseRobot
            // @JvmField val backLeftMotor   : DcMotorEx FROM BaseRobot
            // @JvmField val frontRightMotor : DcMotorEx FROM BaseRobot
            // @JvmField val backRightMotor  : DcMotorEx FROM BaseRobot

     @JvmField val intakeLiftLeft          : DcMotorEx     = hwInit("intakeLiftLeft")
     @JvmField val intakeLiftRight         : DcMotorEx     = hwInit("intakeLiftRight")
     @JvmField val intakePivot             : DcMotorEx     = hwInit("intakePivotMotor")

     @JvmField val odometryLeftAsMotor     : DcMotorEx     = hwInit("odometryLeftAsMotor")

    override val leftOdometryModule: OdometryModule?  = null
    override val rightOdometryModule: OdometryModule? = null
    override val rearOdometryModule: OdometryModule?  = null

    // Servo/PWM Variables
    @JvmField val foundationGrabFrontLeft : Servo                 = hwInit("foundationGrabFrontLeft")
    @JvmField val foundationGrabFrontRight: Servo                 = hwInit("foundationGrabFrontRight")

    @JvmField val autoBlockGrabFront      : Servo                 = hwInit("autoBlockGrabFront")
    @JvmField val autoBlockPivotFront     : Servo                 = hwInit("autoBlockPivotFront")
    @JvmField val autoBlockGrabRear       : Servo                 = hwInit("autoBlockGrabRear")
    @JvmField val autoBlockPivotRear      : Servo                 = hwInit("autoBlockPivotRear")

    @JvmField val intakeBlockGrabberServo : Servo                 = hwInit("intakeBlockGrabber")
    @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")

    // Expansion Hub Variables
//    @JvmField val expansionhubs           : List<LynxModule>      = hardwareMap.getAll(LynxModule::class.java).apply { forEach {it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO} }

    // IMU Variables
    @JvmField val imuControllerA          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'A')
    @JvmField val imuControllerB          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'B')

//     Robot Systems Variables
     @JvmField val foundationGrabbersFront : FoundationGrabbers = FoundationGrabbers(leftServo = foundationGrabFrontLeft, leftLock = 0.00, leftUnlock = 0.49, leftMid = 0.16,
        rightServo = foundationGrabFrontRight, rightLock = 0.25, rightUnlock = 0.80, rightMid = 0.41)

     @JvmField val intakeBlockGrabber      : IntakeBlockGrabber = IntakeBlockGrabber(intakeBlockGrabberServo, 0.00, 0.30, 0.8)

     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerA,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 TwoWheelOdometryLocalizer(odometryLeftAsMotor, intakeLiftLeft, imuControllerA))

     @JvmField val autoBlockIntakeFront    : AutoBlockIntake = AutoBlockIntake(
             pivotServo = autoBlockPivotFront,  pivot18 = 0.13, pivotMid = 0.28, pivotVertical = 0.18, pivotPickup = 0.48,
             grabberServo = autoBlockGrabFront, grabUp = 0.99, grabMid = 0.64, grabPickup = 0.37)
     @JvmField val autoBlockIntakeRear     : AutoBlockIntake = AutoBlockIntake(
            pivotServo = autoBlockPivotRear,  pivot18 = 0.60, pivotMid = 0.43, pivotVertical = 0.55, pivotPickup = 0.23,
            grabberServo = autoBlockGrabRear, grabUp = 0.95, grabMid = 0.65, grabPickup = 0.30)

     @JvmField val odometryModuleLeft = OdometryModule(odometryLeftAsMotor)
     @JvmField val odometryModuleRear = OdometryModule(intakeLiftLeft)
}
