package org.firstinspires.ftc.teamcode.testopmodes.gen2tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot;

@Config
@Autonomous(group="Test")
public class FoundationGrabbingTest extends LinearOpMode {

    public static double x = 0.0;
    public static double y = 0.0;
    public static double z = 0.0;

    public static int timeMs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ExtMisumiRobot robot = new ExtMisumiRobot(hardwareMap);
        robot.foundationGrabbersFront.lock();
        waitForStart();
        robot.holonomic.runWithoutEncoder(x, y, z);
        sleep(timeMs);
        robot.holonomic.stop();

    }
}
