package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile
import org.firstinspires.ftc.teamcode.library.functions.FieldSide
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemBoolean


import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemEnum
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemInteger

class AutoMenuControllerIterative(telemetry: Telemetry) {
    @JvmField val menu = IterableTelemetryMenu(telemetry)

    private val i_startingPosition = MenuItemEnum("sp", "Position", FieldSide.LOADING_ZONE, FieldSide.WAFFLE_SIDE)
    private val i_allianceColor = MenuItemEnum("alliance", "Alliance", AllianceColor.RED, AllianceColor.BLUE)
    private val i_parkNearDS = MenuItemBoolean("parkneards", "Park closer to DS", false);
    private val i_musicFile = MenuItemEnum("music", "Music", ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY, ExtMusicFile.CRABRAVE, ExtMusicFile.BRADTHECHEMIST, ExtMusicFile.TETRIS, ExtMusicFile.MEGALOVANIA, ExtMusicFile.PACMAN, ExtMusicFile.PIZZATIME)
    private val i_parkOnly = MenuItemBoolean("parkonly", "Park ONLY", false)
    private val i_delayBeforeParking = MenuItemInteger("delay", "Delay Before Parking", 0, 0, 20)

    init {
        menu.add(i_musicFile, i_startingPosition, i_parkNearDS, i_allianceColor, i_parkOnly, i_delayBeforeParking)
    }
    val musicFile: ExtMusicFile
        get() = i_musicFile.value
    val startingPosition: FieldSide
        get() = i_startingPosition.value
    val allianceColor : AllianceColor
        get() = i_allianceColor.value
    val parkNearDS : Boolean
        get() = i_parkNearDS.value
    val parkOnly : Boolean
        get() = i_parkOnly.value
    val delayBeforeParking : Int
        get() = i_delayBeforeParking.value

}
//
//class AutoMenuControllerReflectiveIterative(telemetry: Telemetry) {
//    @JvmField var musicFile : ExtMusicFile = ExtMusicFile.NONE
//    @JvmField var driveTime : Int = 1000
//    @JvmField val menu = IterableReflectiveTelemetryMenu(telemetry,
//            MenuItemEnum("Music",::musicFile, ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY, ExtMusicFile.BRADTHECHEMIST),
//            MenuItemInteger("Driving Time", ::driveTime, 0, 5000, 500))
//
//}
//
//class AutoMenuControllerDelegated(telemetry: Telemetry) {
//    @JvmField val menu = DelegatedTelemetryMenu(telemetry)
//    var musicFile : ExtMusicFile by MenuItemEnumDelegate(menu, "Music",ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY)
////    var angle : Int by MenuItemIntDelegate(menu, "Angle", 0, 0, 180, 30)
//    var driveTime : Int by MenuItemIntDelegate(menu, "time", 0,0, 5000, 500)
//
//}