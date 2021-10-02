package org.firstinspires.ftc.teamcode.library.robot.robotcore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.InvocationTargetException;

@Config
public class RobotProvider {
    public static RobotType selectedRobot = RobotType.PUSHBOT;

    public static BaseRobot providePresetRobot(HardwareMap hardwareMap) {
        BaseRobot robot;
        System.out.println("Robot request initiated!");
        try {
            System.out.println("creating " + selectedRobot.robotClass.getName());
            robot = selectedRobot.robotClass.getConstructor(HardwareMap.class).newInstance(hardwareMap);
            System.out.println("sending instantiated" + selectedRobot.robotClass.getName() + " to source");
        } catch (InvocationTargetException e) {
            throw(new RuntimeException("Cannot create robot device via reflection" +
                    ((e.getCause() instanceof IllegalArgumentException)?": "+e.getCause().getMessage():"!"), e));
        } catch (Exception e) {
            System.out.println("INSTANTIATION FAILED!!! Creating ExtPushBot instead!");
            robot = null;
            e.printStackTrace();
        }
        return robot;
    }

    public enum RobotType {
        RINGPLACE(ExtRingPlaceBot.class),
        FLYWHEEL(ExtZoomBot.class),
        PUSHBOT(ExtPushBot202102.class);

        public Class<? extends BaseRobot> robotClass;

        RobotType(Class<? extends BaseRobot> robotClass) {
            this.robotClass = robotClass;
        }
    }

}
