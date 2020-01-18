package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.foundation.FoundationGrabbers
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.intake.IntakeBlockGrabber
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.intake.AutoBlockIntake

open class BasicRobot(protected val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor          : DcMotor               = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor           : DcMotor               = hwInit("backLeftMotor")
     @JvmField val frontRightMotor         : DcMotor               = hwInit("frontRightMotor")
     @JvmField val backRightMotor          : DcMotor               = hwInit("backRightMotor")
     @JvmField val testMotor               : DcMotorEx             = hwInit("frontRightMotor")

     @JvmField val intakeBlockManipulator  : DcMotor               = hwInit("intakeBlockManipulator")
     @JvmField val intakePivotMotor        : DcMotor               = hwInit("intakePivotMotor")

     protected val rearOdometryAsMotor     : DcMotor               = hwInit("rearOdometryModule")
     protected val leftOdometryAsMotor     : DcMotor               = hwInit("leftOdometryModule")
     protected val rightOdometryAsMotor    : DcMotor               = intakePivotMotor

    // Servo Variables
     private   val leftFoundationServo     : Servo                 = hwInit("leftFoundationServo")
     private   val rightFoundationServo    : Servo                 = hwInit("rightFoundationServo")
     private   val intakeGrabberServo      : Servo                 = hwInit("intakeGrabberServo")
     private   val autoBlockGrabber        : Servo                 = hwInit("autoBlockGrabber")
     private   val autoBlockPivot          : Servo                 = hwInit("autoBlockPivot")
    // IMU Variables
//     @JvmField val imuA                    : BNO055IMU             = hwInit("imuA")
//     @JvmField val imuB                    : BNO055IMU             = hwInit("imuB")

    // Analog Input Variables
     @JvmField val intakePivotPotentiometer: AnalogInput           = hwInit("potentiometer")

    // Color/Distance Sensor Variables
//     @JvmField val intakeBlockCSensor      : ColorSensor           = hwInit("intakeBlockSensor")
//     @JvmField val intakeBlockDSensor      : DistanceSensor        = hwInit("intakeBlockSensor")

     @JvmField val leftColorSensor         : ColorSensor           = hwInit("leftColorSensor")
     @JvmField val rightColorSensor        : ColorSensor           = hwInit("rightColorSensor")

     @JvmField val leftColorDistanceSensor : DistanceSensor        = hwInit("leftColorSensor")
     @JvmField val rightColorDistanceSensor: DistanceSensor        = hwInit("rightColorSensor")

     @JvmField val frontDistanceSensor     : ModernRoboticsI2cRangeSensor = hwInit("frontDistanceSensor")
     @JvmField val leftDistanceSensor      : ModernRoboticsI2cRangeSensor = hwInit("leftDistanceSensor")
     @JvmField val rightDistanceSensor     : ModernRoboticsI2cRangeSensor = hwInit("rightDistanceSensor")


    // Robot Systems Variables
     @JvmField val holonomic               : Holonomic = Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
     @JvmField val foundationGrabbers      : FoundationGrabbers    = FoundationGrabbers(leftFoundationServo, rightFoundationServo)
     @JvmField val intakeBlockGrabber      : IntakeBlockGrabber    = IntakeBlockGrabber(intakeGrabberServo)
     @JvmField val autoBlockIntake         : AutoBlockIntake       = AutoBlockIntake(autoBlockPivot, autoBlockGrabber)
     @JvmField val rearOdometry            : OdometryModule        = OdometryModule(rearOdometryAsMotor, false)
     @JvmField val leftOdometry            : OdometryModule        = OdometryModule(leftOdometryAsMotor, true)
     @JvmField val rightOdometry           : OdometryModule        = OdometryModule(rightOdometryAsMotor, true)
     @JvmField val extraOdometry           : OdometryModule        = OdometryModule(intakeBlockManipulator, false)
    //     @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")

    protected inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
