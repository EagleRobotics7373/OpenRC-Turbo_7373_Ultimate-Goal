package org.firstinspires.ftc.teamcode.library.robot.systems.intakegen2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class IntakeConstants {
    public static PIDCoefficients RAISE_COEFFS = new PIDCoefficients(5.0, 0.0, 25.0);
    public static boolean reverseMotorOutput = true;
}
