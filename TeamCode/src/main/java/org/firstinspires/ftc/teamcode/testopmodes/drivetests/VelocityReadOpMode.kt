package org.firstinspires.ftc.teamcode.testopmodes.drivetests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp
class VelocityReadOpMode : LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get(DcMotorEx::class.java, "singleMotor")
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        waitForStart()
        while (opModeIsActive()) {
            val packet = TelemetryPacket()

            packet.put("velo", motor.velocity)
            packet.put("pos", motor.currentPosition)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}