package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MultipleTelemetryContainer {
    public static MultipleTelemetry createMultipleTelemetry(Telemetry telemetry){
        return new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
