package com.example.ftc_depth_ai.ftc

import android.graphics.Canvas
import android.util.Size
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName
import org.opencv.core.Mat
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera.AsyncCameraCloseListener
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraRotation
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvInternalCamera
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvSwitchableWebcam
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam
import com.example.vision.ftcdepthai.openftc.easyopencv.TimestampedOpenCvPipeline
import kotlin.concurrent.Volatile

/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
class VisionPortalImpl(
    camera: CameraName?,
    cameraMonitorViewId: Int,
    autoPauseCameraMonitor: Boolean,
    protected val cameraResolution: Size?,
    protected val webcamStreamFormat: StreamFormat,
    protected var processors: Array<VisionProcessor?>
) : VisionPortal() {
    protected var camera: com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera? = null

    @Volatile
    override var cameraState = CameraState.CAMERA_DEVICE_CLOSED
        protected set

    @Volatile
    protected var processorsEnabled: BooleanArray

    @Volatile
    protected var calibration: CameraCalibration? = null
    protected val autoPauseCameraMonitor: Boolean
    protected val userStateMtx = Any()
    protected var captureNextFrame: String? = null
    protected val captureFrameMtx = Any()

    init {
        processorsEnabled = BooleanArray(processors.size)
        for (i in processors.indices) {
            processorsEnabled[i] = true
        }
        this.autoPauseCameraMonitor = autoPauseCameraMonitor
        createCamera(camera, cameraMonitorViewId)
        startCamera()
    }

    protected fun startCamera() {
        checkNotNull(camera) { "This should never happen" }
        requireNotNull(
            cameraResolution // was the user a silly silly
        ) { "parameters.cameraResolution == null" }
        camera!!.setViewportRenderer(com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera.ViewportRenderer.NATIVE_VIEW)
        if (camera !is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam) {
            camera!!.setViewportRenderingPolicy(com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW)
        }
        cameraState = CameraState.OPENING_CAMERA_DEVICE
        camera!!.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                cameraState = CameraState.CAMERA_DEVICE_READY
                cameraState = CameraState.STARTING_STREAM
                if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam) {
                    (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam).startStreaming(
                        cameraResolution.width,
                        cameraResolution.height,
                        CAMERA_ROTATION,
                        webcamStreamFormat.eocvStreamFormat
                    )
                } else {
                    camera!!.startStreaming(
                        cameraResolution.width,
                        cameraResolution.height,
                        CAMERA_ROTATION
                    )
                }
                if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam) {
                    val identity = (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam).calibrationIdentity
                    if (identity != null) {
                        calibration = CameraCalibrationHelper.getInstance().getCalibration(
                            identity,
                            cameraResolution.width,
                            cameraResolution.height
                        )
                    }
                }
                camera!!.setPipeline(ProcessingPipeline())
                cameraState = CameraState.STREAMING
            }

            override fun onError(errorCode: Int) {
                cameraState = CameraState.ERROR
                RobotLog.ee("VisionPortalImpl", "Camera opening failed.")
            }
        })
    }

    protected fun createCamera(cameraName: CameraName?, cameraMonitorViewId: Int) {
        if (cameraName == null) // was the user a silly silly
        {
            throw IllegalArgumentException("parameters.camera == null")
        } else if (cameraName.isWebcam) // Webcams
        {
            camera = if (cameraMonitorViewId != 0) {
                com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance()
                    .createWebcam(cameraName as WebcamName?, cameraMonitorViewId)
            } else {
                com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance().createWebcam(cameraName as WebcamName?)
            }
        } else if (cameraName.isCameraDirection) // Internal cameras
        {
            camera = if (cameraMonitorViewId != 0) {
                com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance().createInternalCamera(
                    if ((cameraName as BuiltinCameraName).cameraDirection == BuiltinCameraDirection.BACK) com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvInternalCamera.CameraDirection.BACK else com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvInternalCamera.CameraDirection.FRONT,
                    cameraMonitorViewId
                )
            } else {
                com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance().createInternalCamera(
                    if ((cameraName as BuiltinCameraName).cameraDirection == BuiltinCameraDirection.BACK) com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvInternalCamera.CameraDirection.BACK else com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvInternalCamera.CameraDirection.FRONT
                )
            }
        } else if (cameraName.isSwitchable) {
            val switchableCameraName = cameraName as SwitchableCameraName
            if (switchableCameraName.allMembersAreWebcams()) {
                val members = switchableCameraName.members
                val webcamNames = arrayOfNulls<WebcamName>(members.size)
                for (i in members.indices) {
                    webcamNames[i] = members[i] as WebcamName
                }
                camera = if (cameraMonitorViewId != 0) {
                    com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance()
                        .createSwitchableWebcam(cameraMonitorViewId, *webcamNames)
                } else {
                    com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance().createSwitchableWebcam(*webcamNames)
                }
            } else {
                throw IllegalArgumentException("All members of a switchable camera name must be webcam names")
            }
        } else  // ¯\_(ツ)_/¯
        {
            throw IllegalArgumentException("Unknown camera name")
        }
    }

    override fun setProcessorEnabled(processor: VisionProcessor?, enabled: Boolean) {
        var numProcessorsEnabled = 0
        var ok = false
        for (i in processors.indices) {
            if (processor === processors[i]) {
                processorsEnabled[i] = enabled
                ok = true
            }
            if (processorsEnabled[i]) {
                numProcessorsEnabled++
            }
        }
        if (ok) {
            if (autoPauseCameraMonitor) {
                if (numProcessorsEnabled == 0) {
                    camera!!.pauseViewport()
                } else {
                    camera!!.resumeViewport()
                }
            }
        } else {
            throw IllegalArgumentException("Processor not attached to this helper!")
        }
    }

    override fun getProcessorEnabled(processor: VisionProcessor?): Boolean {
        for (i in processors.indices) {
            if (processor === processors[i]) {
                return processorsEnabled[i]
            }
        }
        throw IllegalArgumentException("Processor not attached to this helper!")
    }

    override var activeCamera: WebcamName?
        get() {
            return if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvSwitchableWebcam) {
                (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvSwitchableWebcam).activeCamera
            } else {
                throw UnsupportedOperationException("getActiveCamera is only supported for switchable webcams")
            }
        }
        set(webcamName) {
            if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvSwitchableWebcam) {
                (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvSwitchableWebcam).activeCamera = webcamName
            } else {
                throw UnsupportedOperationException("setActiveCamera is only supported for switchable webcams")
            }
        }

    override fun <T : CameraControl?> getCameraControl(controlType: Class<T>?): T {
        return if (cameraState == CameraState.STREAMING) {
            if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam) {
                (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam).getControl(controlType)
            } else {
                throw UnsupportedOperationException("Getting controls is only supported for webcams")
            }
        } else {
            throw IllegalStateException("You cannot use camera controls until the camera is streaming")
        }
    }

    internal inner class ProcessingPipeline : com.example.vision.ftcdepthai.openftc.easyopencv.TimestampedOpenCvPipeline() {
        init {
            MEMLEAK_DETECTION_ENABLED = false
        }

        override fun init(firstFrame: Mat) {
            for (processor in processors) {
                processor?.init(firstFrame.width(), firstFrame.height(), calibration)
            }
        }

        override fun processFrame(input: Mat, captureTimeNanos: Long): Mat {
            synchronized(captureFrameMtx) {
                if (captureNextFrame != null) {
                    saveMatToDiskFullPath(
                        input,
                        "/sdcard/VisionPortal-" + captureNextFrame + ".png"
                    )
                }
                captureNextFrame = null
            }
            val processorDrawCtxes = arrayOfNulls<Any>(
                processors.size
            ) // cannot re-use frome to frame
            for (i in processors.indices) {
                if (processorsEnabled[i]) {
                    processorDrawCtxes[i] = processors[i]?.processFrame(input, captureTimeNanos)
                }
            }
            requestViewportDrawHook(processorDrawCtxes)
            return input
        }

        override fun onDrawFrame(
            canvas: Canvas,
            onscreenWidth: Int,
            onscreenHeight: Int,
            scaleBmpPxToCanvasPx: Float,
            scaleCanvasDensity: Float,
            userContext: Any
        ) {
            val ctx = userContext as Array<Any>
            for (i in processors.indices) {
                if (processorsEnabled[i]) {
                    processors[i]?.onDrawFrame(
                        canvas,
                        onscreenWidth,
                        onscreenHeight,
                        scaleBmpPxToCanvasPx,
                        scaleCanvasDensity,
                        ctx[i]
                    )
                }
            }
        }
    }

    override fun saveNextFrameRaw(filepath: String?) {
        synchronized(captureFrameMtx) { captureNextFrame = filepath }
    }

    override fun stopStreaming() {
        synchronized(userStateMtx) {
            if (cameraState == CameraState.STREAMING || cameraState == CameraState.STARTING_STREAM) {
                cameraState = CameraState.STOPPING_STREAM
                Thread(Runnable {
                    synchronized(userStateMtx) {
                        camera!!.stopStreaming()
                        cameraState = CameraState.CAMERA_DEVICE_READY
                    }
                }).start()
            } else if ((cameraState == CameraState.STOPPING_STREAM
                        ) || (cameraState == CameraState.CAMERA_DEVICE_READY
                        ) || (cameraState == CameraState.CLOSING_CAMERA_DEVICE)
            ) {
                // be idempotent
            } else {
                throw RuntimeException("Illegal CameraState when calling stopStreaming()")
            }
        }
    }

    override fun resumeStreaming() {
        synchronized(userStateMtx) {
            if (cameraState == CameraState.CAMERA_DEVICE_READY || cameraState == CameraState.STOPPING_STREAM) {
                cameraState = CameraState.STARTING_STREAM
                Thread(Runnable {
                    synchronized(userStateMtx) {
                        if (camera is com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam) {
                            (camera as com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam).startStreaming(
                                cameraResolution!!.getWidth(),
                                cameraResolution.getHeight(),
                                CAMERA_ROTATION,
                                webcamStreamFormat.eocvStreamFormat
                            )
                        } else {
                            camera!!.startStreaming(
                                cameraResolution!!.getWidth(),
                                cameraResolution.getHeight(),
                                CAMERA_ROTATION
                            )
                        }
                        cameraState = CameraState.STREAMING
                    }
                }).start()
            } else if ((cameraState == CameraState.STREAMING
                        ) || (cameraState == CameraState.STARTING_STREAM
                        ) || (cameraState == CameraState.OPENING_CAMERA_DEVICE)
            ) // we start streaming automatically after we open
            {
                // be idempotent
            } else {
                throw RuntimeException("Illegal CameraState when calling stopStreaming()")
            }
        }
    }

    override fun stopLiveView() {
        val cameraSafe = camera
        if (cameraSafe != null) {
            camera!!.pauseViewport()
        }
    }

    override fun resumeLiveView() {
        val cameraSafe = camera
        if (cameraSafe != null) {
            camera!!.resumeViewport()
        }
    }

    override val fps: Float
        get() {
            val cameraSafe = camera
            return if (cameraSafe != null) {
                cameraSafe.fps
            } else {
                0F
            }
        }

    override fun close() {
        synchronized(userStateMtx) {
            cameraState = CameraState.CLOSING_CAMERA_DEVICE
            if (camera != null) {
                camera!!.closeCameraDeviceAsync(AsyncCameraCloseListener {
                    cameraState = CameraState.CAMERA_DEVICE_CLOSED
                })
            }
            camera = null
        }
    }

    companion object {
        protected val CAMERA_ROTATION = com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraRotation.SENSOR_NATIVE
    }
}