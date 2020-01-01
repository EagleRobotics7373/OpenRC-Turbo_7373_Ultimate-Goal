package org.firstinspires.ftc.teamcode.testopmodes.functiontests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveMenuItemBoolean
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveMenuItemEnum
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveMenuItemInteger
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.ReflectiveTelemetryMenu

@Autonomous(group="Test")
class ReflectiveMenuTest : LinearOpMode() {
    var sampleInteger = 0
    var sampleBoolean = false
    var sampleAllianceColor = AllianceColor.RED

    override fun runOpMode() {
        val menu = ReflectiveTelemetryMenu(telemetry,
                ReflectiveMenuItemInteger("Sample Integer", ::sampleInteger, 0, 10, 1),
                ReflectiveMenuItemBoolean("Sample Boolean", ::sampleBoolean),
                ReflectiveMenuItemEnum("Sample Alliance Color", ::sampleAllianceColor, AllianceColor.RED, AllianceColor.BLUE))

        while (!isStarted) {
            if (gamepad1.dpad_up) {
                menu.previousItem()
                while (gamepad1.dpad_up and !isStopRequested);
            }
            else if (gamepad1.dpad_down) {
                menu.nextItem()
                while (gamepad1.dpad_down and !isStopRequested);
            }
            else if (gamepad1.dpad_left) {
                menu.iterateBackward()
                while (gamepad1.dpad_left and !isStopRequested);
            }
            else if (gamepad1.dpad_right) {
                menu.iterateForward()
                while (gamepad1.dpad_right and !isStopRequested);
            }
        }
    }



}
