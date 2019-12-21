package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.FoundationGrabbers
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.IntakeBlockGrabber
import org.firstinspires.ftc.teamcode.library.robot.systems.OdometryModule

class BasicRobot(private val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor          : DcMotor               = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor           : DcMotor               = hwInit("backLeftMotor")
     @JvmField val frontRightMotor         : DcMotor               = hwInit("frontRightMotor")
     @JvmField val backRightMotor          : DcMotor               = hwInit("backRightMotor")

     @JvmField val intakeBlockManipulator  : DcMotor               = hwInit("intakeBlockManipulator")
     @JvmField val intakePivotMotor        : DcMotor               = hwInit("intakePivotMotor")

     private   val odometryXAxisAsMotor    : DcMotor               = hwInit("odometryXAxis")

    // Servo Variables
     private   val leftFoundationServo     : Servo                 = hwInit("leftFoundationServo")
     private   val rightFoundationServo    : Servo                 = hwInit("rightFoundationServo")
     private   val intakeGrabberServo      : Servo                 = hwInit("intakeGrabberServo")

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
     @JvmField val holonomic               : Holonomic              = Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
     @JvmField val foundationGrabbers      : FoundationGrabbers     = FoundationGrabbers(leftFoundationServo, rightFoundationServo)
     @JvmField val intakeBlockGrabber      : IntakeBlockGrabber     = IntakeBlockGrabber(intakeGrabberServo)
     @JvmField val odometryXAxis           : OdometryModule         = OdometryModule(odometryXAxisAsMotor)
    //     @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")
    private inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
