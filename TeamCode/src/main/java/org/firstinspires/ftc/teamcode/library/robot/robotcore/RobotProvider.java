package org.firstinspires.ftc.teamcode.library.robot.robotcore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class RobotProvider {
    public static RobotType selectedRobot = RobotType.PUSHBOT;
    public static boolean useTwoWheelOdometry = true;

    public static BaseRobot providePresetRobot(HardwareMap hardwareMap) {
        BaseRobot robot;
        try {
            robot = selectedRobot.robotClass.getConstructor(HardwareMap.class).newInstance(hardwareMap);
        } catch (Exception e) {
            robot = new ExtPushBot(hardwareMap);
            e.printStackTrace();
        }
        return robot;
    }

    public enum RobotType {
        MISUMI(ExtPushBot.class), PUSHBOT(ExtPushBot.class);

        public Class<? extends BaseRobot> robotClass;

        RobotType(Class<? extends BaseRobot> robotClass) {
            this.robotClass = robotClass;
        }
    }

}
