package org.firstinspires.ftc.teamcode.testopmodes.sensortests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.openftc.revextensions2.ExpansionHubEx;

public class VoltageSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("voltage", getBatteryVoltage());
        telemetry.update();
        sleep(1000);
    }

    private double getBatteryVoltageOld() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor: hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private double getBatteryVoltage() {
        ExpansionHubEx expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        return expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
    }

}
