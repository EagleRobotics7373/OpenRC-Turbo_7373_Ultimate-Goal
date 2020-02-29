package org.firstinspires.ftc.teamcode.testopmodes.functiontests

import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile
import java.io.File

@TeleOp(group = "Test")
class SoundPlayerTest : LinearOpMode() {
    override fun runOpMode() {

        waitForStart()

        val context = hardwareMap.appContext
        val player = SoundPlayer.getInstance()
        val musicFile = ExtMusicFile.BADPIGGIES
        val playingParams = SoundPlayer.PlaySoundParams().also {
            it.volume = 1.0f
            it.loopControl = -1
            it.rate = 1.0f
        }

        while (opModeIsActive()) {
            player.play(context, File(musicFile.fileName), 100.0f, -1, 1.0f)
            player.startPlaying(context, File(musicFile.fileName), playingParams, null, null)
        }
    }
}