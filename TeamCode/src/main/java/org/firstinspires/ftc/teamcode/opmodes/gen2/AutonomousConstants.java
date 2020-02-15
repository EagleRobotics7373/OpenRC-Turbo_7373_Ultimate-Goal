package org.firstinspires.ftc.teamcode.opmodes.gen2;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;

@Config
public class AutonomousConstants {
    public static AllianceColor _allianceColor = AllianceColor.BLUE;
    public static FieldSide _startingPosition = FieldSide.LOADING_ZONE;
    public static ExtMusicFile _musicFile = ExtMusicFile.CREEPER_AWMAN;
    public static boolean _parkOnly = false;
    public static int _delayBeforeParking = 0;
    public static boolean _foundationSwivel = false;
    public static boolean _doFoundationPull = true;
    public static boolean _doParkAtEnd = true;
    public static boolean _liftLowerOnly = false;

    // other things
    public static double RR_NEXT_TO_STONE_Y = 35.25;
    public static double RR_AGAINST_BRIDGE_Y = 42.5;
    public static double RR_AGAINST_FOUNDATION_Y = 32.0;
    public static int NUM_STONES = 4;
}
