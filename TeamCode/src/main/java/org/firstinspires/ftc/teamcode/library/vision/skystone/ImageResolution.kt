package org.firstinspires.ftc.teamcode.library.vision.skystone

enum class ImageResolution(val width: Int, val height: Int, val scale: Double = 1.0) {
    R_320x240(320,240, 0.44),
    R_640x480(640, 480, 0.88),
    R_720x480(720, 480, 1.00),
    R_1280x720(1280, 720, 1.78);

    override fun toString(): String =
            "${width}x${height};${scale}"
}