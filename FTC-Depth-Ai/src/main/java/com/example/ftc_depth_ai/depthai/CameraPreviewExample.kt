package com.example.ftc_depth_ai.depthai

import org.bytedeco.depthai.ColorCameraProperties
import org.bytedeco.depthai.Device
import org.bytedeco.depthai.Pipeline
import org.bytedeco.opencv.global.opencv_core
import org.bytedeco.opencv.global.opencv_highgui
import org.bytedeco.opencv.opencv_core.Mat

// Inludes common necessary includes for development using depthai library
object CameraPreviewExample {
    fun createCameraPipeline(): Pipeline {
        val p = Pipeline()
        val colorCam = p.createColorCamera()
        val xlinkOut = p.createXLinkOut()
        xlinkOut.setStreamName("preview")
        colorCam.setPreviewSize(300, 300)
        colorCam.resolution = ColorCameraProperties.SensorResolution.THE_1080_P
        colorCam.interleaved = true

        // Link plugins CAM -> XLINK
        colorCam.preview().link(xlinkOut.input())
        return p
    }

    @JvmStatic
    fun main(args: Array<String>) {
        val p = createCameraPipeline()

        // Start the pipeline
        val d = Device()
        print("Connected cameras: ")
        val cameras = d.connectedCameras
        for (i in 0 until cameras.limit()) {
            print(cameras[i].toString() + " ")
        }
        println()

        // Start the pipeline
        d.startPipeline(p)
        var frame: Mat?
        val preview = d.getOutputQueue("preview")
        while (true) {
            val imgFrame = preview.imgFrame
            if (imgFrame != null) {
                System.out.printf("Frame - w: %d, h: %d\n", imgFrame.width, imgFrame.height)
                frame = Mat(imgFrame.height, imgFrame.width, opencv_core.CV_8UC3, imgFrame.data)
                opencv_highgui.imshow("preview", frame)
                val key = opencv_highgui.waitKey(1)
                if (key == 'q'.code) {
                    System.exit(0)
                }
            } else {
                println("Not ImgFrame")
            }
        }
    }
}