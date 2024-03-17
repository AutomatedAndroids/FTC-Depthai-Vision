///*
// * Copyright (c) 2023 FIRST
// *
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to
// * endorse or promote products derived from this software without specific prior
// * written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//package com.example.ftc_depth_ai.ftc.tfod.deprecated
//
//import android.graphics.Bitmap
//import android.graphics.Canvas
//import org.firstinspires.ftc.robotcore.external.ClassFactory
//import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation
//import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator
//import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer
//import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
//import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
//import org.opencv.android.Utils
//import org.opencv.core.Mat

//@Deprecated("This class has been disabled to protect from internal errors, such as classpath. please use the traditional system.")
//internal class TfodProcessorImpl(protected val parameters: TfodParameters) : TfodProcessor(),
//    FrameGenerator {
//    protected val tfObjectDetector: TFObjectDetector
//    protected val frameConsumerLock = Any()
//    protected var frameConsumer: FrameConsumer? = null
//        set(value) {
//            synchronized(frameConsumerLock) {
//                field = value
//                if (frameConsumer != null) {
//                    bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
//                    frameConsumer?.init(bitmap)
//                }
//            }
//        }
//    protected var bitmap: Bitmap? = null
//    protected var width = 0
//    protected var height = 0
//    protected var fx = 0f
//    protected var fy = 100f // dummy
//
//    init {
//        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(
//            parameters, this
//        )
//    }
//
//    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
//        this.width = width
//        this.height = height
//        if (calibration != null) {
//            fx = calibration.focalLengthX
//            fy = calibration.focalLengthY
//        }
//        tfObjectDetector.activate()
//    }
//
//    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
//        var frameConsumerSafe: FrameConsumer
//        var bitmapSafe: Bitmap?
//        synchronized(frameConsumerLock) {
//            if (frameConsumer == null) {
//                return null
//            }
//            frameConsumerSafe = frameConsumer as FrameConsumer
//            bitmapSafe = bitmap
//        }
//        Utils.matToBitmap(frame, bitmapSafe)
//        return frameConsumerSafe.processFrame()
//    }
//
//    override fun onDrawFrame(
//        canvas: Canvas?,
//        onscreenWidth: Int,
//        onscreenHeight: Int,
//        scaleBmpPxToCanvasPx: Float,
//        scaleCanvasDensity: Float,
//        userContext: Any?
//    ) {
//        if (userContext != null) {
//            (userContext as CanvasAnnotator).draw(
//                canvas,
//                onscreenWidth,
//                onscreenHeight,
//                scaleBmpPxToCanvasPx,
//                scaleCanvasDensity
//            )
//        }
//    }
//
//    override fun getCameraInformation(): CameraInformation {
//        return CameraInformation(width, height, 0, fx, fy)
//    }
//
//    override fun setFrameConsumer(frameConsumer: FrameConsumer?) {
//        TODO("Not yet implemented")
//    }
//
//    override fun setMinResultConfidence(minResultConfidence: Float) {
//        tfObjectDetector.setMinResultConfidence(minResultConfidence)
//    }
//
//    override fun setClippingMargins(left: Int, top: Int, right: Int, bottom: Int) {
//        tfObjectDetector.setClippingMargins(left, top, right, bottom)
//    }
//
//    override fun setZoom(magnification: Double) {
//        tfObjectDetector.setZoom(magnification)
//    }
//
//    override val recognitions: List<Recognition?>?
//        get() = tfObjectDetector.recognitions
//    override val freshRecognitions: List<Recognition?>?
//        get() = tfObjectDetector.updatedRecognitions
//
//    override fun shutdown() {
//        tfObjectDetector.shutdown()
//    }
//}