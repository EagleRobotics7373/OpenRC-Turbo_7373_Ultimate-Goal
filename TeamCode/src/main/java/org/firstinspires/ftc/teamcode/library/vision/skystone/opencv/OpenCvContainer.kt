package org.firstinspires.ftc.teamcode.library.vision.skystone.opencv

import org.firstinspires.ftc.teamcode.library.robot.systems.Stoppable
import org.firstinspires.ftc.teamcode.library.vision.skystone.ImageResolution
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class OpenCvContainer<out Pipeline : OpenCvPipeline>(
        val camera: OpenCvCamera,
        val pipeline: Pipeline,
        val resolution: ImageResolution,
        rotation: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT) : Stoppable {

    init {
        camera.setPipeline(pipeline)
        camera.openCameraDevice()
        camera.startStreaming(resolution.width, resolution.height, rotation)
        if (pipeline is PixelStatsPipeline) pipeline.resolution = resolution
    }

    override fun stop() = camera.stopStreaming()
}