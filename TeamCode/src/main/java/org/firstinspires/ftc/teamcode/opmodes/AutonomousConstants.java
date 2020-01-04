package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;
import org.firstinspires.ftc.teamcode.library.functions.Position;

@Config
public class AutonomousConstants {
    public static AllianceColor _allianceColor = AllianceColor.RED;
    public static FieldSide _startingPosition = FieldSide.WAFFLE_SIDE;
    public static ExtMusicFile _musicFile = ExtMusicFile.NONE;
    public static boolean _parkOnly = false;
    public static int _delayBeforeParking = 0;
    public static boolean _foundationSwivel = false;
}
