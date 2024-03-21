package com.example.ftc_depth_ai.ftc

import android.util.Size
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.ViewportSplitMethod
import com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam

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
abstract class VisionPortal {
    /**
     * StreamFormat is only applicable if using a webcam
     */
    enum class StreamFormat(public val eocvStreamFormat: com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam.StreamFormat) {
        /** The only format that was supported historically; it is uncompressed but
         * chroma subsampled and uses lots of bandwidth - this limits frame rate
         * at higher resolutions and also limits the ability to use two cameras
         * on the same bus to lower resolutions
         */
        YUY2(com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam.StreamFormat.YUY2),

        /** Compressed motion JPEG stream format; allows for higher resolutions at
         * full frame rate, and better ability to use two cameras on the same bus.
         * Requires extra CPU time to run decompression routine.
         */
        MJPEG(com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvWebcam.StreamFormat.MJPEG)
    }

    /**
     * If you are using multiple vision portals with live previews concurrently,
     * you need to split up the screen to make room for both portals
     */
    enum class MultiPortalLayout(val viewportSplitMethod: ViewportSplitMethod) {
        /**
         * Divides the screen vertically
         */
        VERTICAL(ViewportSplitMethod.VERTICALLY),

        /**
         * Divides the screen horizontally
         */
        HORIZONTAL(ViewportSplitMethod.HORIZONTALLY)
    }

    class Builder {
        private var camera: CameraName? = null
        private var liveViewContainerId: Int =
            VisionPortal.Companion.DEFAULT_VIEW_CONTAINER_ID // 0 == none
        private var autoStopLiveView = true
        private var cameraResolution = Size(640, 480)
        private var streamFormat: StreamFormat? = null
        private val STREAM_FORMAT_DEFAULT = StreamFormat.YUY2
        private val processors: MutableList<VisionProcessor> = ArrayList<VisionProcessor>()

        /**
         * Configure the portal to use a webcam
         * @param camera the WebcamName of the camera to use
         * @return the [Builder] object, to allow for method chaining
         */
        fun setCamera(camera: CameraName?): VisionPortal.Builder {
            this.camera = camera
            return this
        }

        /**
         * Configure the portal to use an internal camera
         * @param cameraDirection the internal camera to use
         * @return the [Builder] object, to allow for method chaining
         */
        fun setCamera(cameraDirection: BuiltinCameraDirection?): VisionPortal.Builder {
            camera =
                ClassFactory.getInstance().cameraManager.nameFromCameraDirection(cameraDirection)
            return this
        }

        /**
         * Configure the vision portal to stream from the camera in a certain image format
         * THIS APPLIES TO WEBCAMS ONLY!
         * @param streamFormat the desired streaming format
         * @return the [Builder] object, to allow for method chaining
         */
        fun setStreamFormat(streamFormat: StreamFormat?): VisionPortal.Builder {
            this.streamFormat = streamFormat
            return this
        }

        /**
         * Configure the vision portal to use (or not to use) a live camera preview
         * @param enableLiveView whether or not to use a live preview
         * @return the [Builder] object, to allow for method chaining
         */
        fun enableLiveView(enableLiveView: Boolean): VisionPortal.Builder {
            val viewId: Int
            if (enableLiveView) {
                viewId = VisionPortal.Companion.DEFAULT_VIEW_CONTAINER_ID
            } else {
                viewId = 0
            }
            return setLiveViewContainerId(viewId)
        }

        /**
         * Configure whether the portal should automatically pause the live camera
         * view if all attached processors are disabled; this can save computational resources
         * @param autoPause whether to enable this feature or not
         * @return the [Builder] object, to allow for method chaining
         */
        fun setAutoStopLiveView(autoPause: Boolean): VisionPortal.Builder {
            autoStopLiveView = autoPause
            return this
        }

        /**
         * A more advanced version of [.enableLiveView]; allows you
         * to specify a specific view ID to use as a container, rather than just using the default one
         * @param liveViewContainerId view ID of container for live view
         * @return the [Builder] object, to allow for method chaining
         */
        fun setLiveViewContainerId(liveViewContainerId: Int): VisionPortal.Builder {
            this.liveViewContainerId = liveViewContainerId
            return this
        }

        /**
         * Specify the resolution in which to stream images from the camera. To find out what resolutions
         * your camera supports, simply call this with some random numbers (e.g. new Size(4634, 11115))
         * and the error message will provide a list of supported resolutions.
         * @param cameraResolution the resolution in which to stream images from the camera
         * @return the [Builder] object, to allow for method chaining
         */
        fun setCameraResolution(cameraResolution: Size): VisionPortal.Builder {
            this.cameraResolution = cameraResolution
            return this
        }

        /**
         * Send a [VisionProcessor] into this portal to allow it to process camera frames.
         * @param processor the processor to attach
         * @return the [Builder] object, to allow for method chaining
         * @throws RuntimeException if the specified processor is already inside another portal
         */
        fun addProcessor(processor: VisionProcessor): VisionPortal.Builder {
            synchronized(VisionPortal.Builder.Companion.attachedProcessors) {
                if (VisionPortal.Builder.Companion.attachedProcessors.contains(processor)) {
                    throw RuntimeException("This VisionProcessor has already been attached to a VisionPortal, either a different one or perhaps even this same portal.")
                } else {
                    VisionPortal.Builder.Companion.attachedProcessors.add(processor)
                }
            }
            processors.add(processor)
            return this
        }

        /**
         * Send multiple [VisionProcessor]s into this portal to allow them to process camera frames.
         * @param processors the processors to attach
         * @return the [Builder] object, to allow for method chaining
         * @throws RuntimeException if the specified processor is already inside another portal
         */
        fun addProcessors(vararg processors: VisionProcessor?): VisionPortal.Builder {
            for (p in processors) {
                addProcessor(p!!)
            }
            return this
        }

        /**
         * Actually create the [VisionPortal] i.e. spool up the camera and live view
         * and begin sending image data to any attached [VisionProcessor]s
         * @return a configured, ready to use portal
         * @throws RuntimeException if you didn't specify what camera to use
         * @throws IllegalStateException if you tried to set the stream format when not using a webcam
         */
        fun build(): VisionPortal {
            if (camera == null) {
                throw RuntimeException("You can't build a vision portal without setting a camera!")
            }
            if (streamFormat != null) {
                check(!(!camera!!.isWebcam && !camera!!.isSwitchable)) { "setStreamFormat() may only be used with a webcam" }
            } else {
                // Only used with webcams, will be ignored for internal camera
                streamFormat = STREAM_FORMAT_DEFAULT
            }
            val portal: VisionPortal = VisionPortalImpl(
                camera, liveViewContainerId, autoStopLiveView, cameraResolution, streamFormat!!,
                (processors to arrayOfNulls<VisionProcessor>(processors.size)).second
            )

            // Clear this list to allow safe re-use of the builder object
            processors.clear()
            return portal
        }

        companion object {
            // STATIC !
            @JvmStatic private val attachedProcessors: ArrayList<VisionProcessor> =
                ArrayList<VisionProcessor>()
        }
    }

    /**
     * Enable or disable a [VisionProcessor] that is attached to this portal.
     * Disabled processors are not passed new image data and do not consume any computational
     * resources. Of course, they also don't give you any useful data when disabled.
     * This takes effect immediately (on the next frame interval)
     * @param processor the processor to enable or disable
     * @param enabled should it be enabled or disabled?
     * @throws IllegalArgumentException if the processor specified isn't inside this portal
     */
    abstract fun setProcessorEnabled(processor: VisionProcessor?, enabled: Boolean)

    /**
     * Queries whether a given processor is enabled
     * @param processor the processor in question
     * @return whether the processor in question is enabled
     * @throws IllegalArgumentException if the processor specified isn't inside this portal
     */
    abstract fun getProcessorEnabled(processor: VisionProcessor?): Boolean

    /**
     * The various states that the camera may be in at any given time
     */
    enum class CameraState {
        /**
         * The camera device handle is being opened
         */
        OPENING_CAMERA_DEVICE,

        /**
         * The camera device handle has been opened and the camera
         * is now ready to start streaming
         */
        CAMERA_DEVICE_READY,

        /**
         * The camera stream is starting
         */
        STARTING_STREAM,

        /**
         * The camera streaming session is in flight and providing image data
         * to any attached [VisionProcessor]s
         */
        STREAMING,

        /**
         * The camera stream is being shut down
         */
        STOPPING_STREAM,

        /**
         * The camera device handle is being closed
         */
        CLOSING_CAMERA_DEVICE,

        /**
         * The camera device handle has been closed; you must create a new
         * portal if you wish to use the camera again
         */
        CAMERA_DEVICE_CLOSED,

        /**
         * The camera was having a bad day and refused to cooperate with configuration for either
         * opening the device handle or starting the streaming session
         */
        ERROR
    }

    /**
     * Query the current state of the camera (e.g. is a streaming session in flight?)
     * @return the current state of the camera
     */
    abstract val cameraState: VisionPortal.CameraState?
    abstract fun saveNextFrameRaw(filename: String?)

    /**
     * Stop the streaming session. This is an asynchronous call which does not take effect
     * immediately. You may use [.getCameraState] to monitor for when this command
     * has taken effect. If you call [.resumeStreaming] before the operation is complete,
     * it will SYNCHRONOUSLY await completion of the stop command
     *
     * Stopping the streaming session is a good way to save computational resources if there may
     * be long (e.g. 10+ second) periods of match play in which vision processing is not required.
     * When streaming is stopped, no new image data is acquired from the camera and any attached
     * [VisionProcessor]s will lie dormant until such time as [.resumeStreaming] is called.
     *
     * Stopping and starting the stream can take a second or two, and thus is not advised for use
     * cases where instantaneously enabling/disabling vision processing is required.
     */
    abstract fun stopStreaming()

    /**
     * Resume the streaming session if previously stopped by [.stopStreaming]. This is
     * an asynchronous call which does not take effect immediately. If you call [.stopStreaming]
     * before the operation is complete, it will SYNCHRONOUSLY await completion of the resume command.
     *
     * See notes about use case on [.stopStreaming]
     */
    abstract fun resumeStreaming()

    /**
     * Temporarily stop the live view on the RC screen. This DOES NOT affect the ability to get
     * a camera frame on the Driver Station's "Camera Stream" feature.
     *
     * This has no effect if you didn't set up a live view.
     *
     * Stopping the live view is recommended during competition to save CPU resources when
     * a live view is not required for debugging purposes.
     */
    abstract fun stopLiveView()

    /**
     * Start the live view again, if it was previously stopped with [.stopLiveView]
     *
     * This has no effect if you didn't set up a live view.
     */
    abstract fun resumeLiveView()

    /**
     * Get the current rate at which frames are passing through the vision portal
     * (and all processors therein) per second - frames per second
     * @return the current vision frame rate in frames per second
     */
    abstract val fps: Float

    /**
     * Get a camera control handle
     * ONLY APPLICABLE TO WEBCAMS
     * @param controlType the type of control to get
     * @return the requested control
     * @throws UnsupportedOperationException if you are not using a webcam
     */
    abstract fun <T : CameraControl?> getCameraControl(controlType: Class<T>?): T
    /**
     * Returns the name of the currently active camera
     * ONLY APPLIES IF USING A SWITCHABLE WEBCAM
     * @return the name of the currently active camera
     * @throws UnsupportedOperationException if you are not using a switchable webcam
     */
    /**
     * Switches the active camera to the indicated camera.
     * ONLY APPLICABLE IF USING A SWITCHABLE WEBCAM
     * @param webcamName the name of the to-be-activated camera
     * @throws UnsupportedOperationException if you are not using a switchable webcam
     */
    abstract var activeCamera: WebcamName?

    /**
     * Teardown everything prior to the end of the OpMode (perhaps to save resources) at which point
     * it will be torn down automagically anyway.
     *
     * This will stop all vision related processing, shut down the camera, and remove the live view.
     * A closed portal may not be re-opened: if you wish to use the camera again, you must make a new portal
     */
    abstract fun close()

    companion object {
        @JvmStatic val DEFAULT_VIEW_CONTAINER_ID = AppUtil.getDefContext().resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            AppUtil.getDefContext().packageName
        )

        /**
         * Split up the screen for using multiple vision portals with live views simultaneously
         * @param numPortals the number of portals to create space for on the screen
         * @param mpl the methodology for laying out the multiple live views on the screen
         * @return an array of view IDs, whose elements may be passed to [Builder.setLiveViewContainerId]
         */
        @JvmStatic fun makeMultiPortalView(numPortals: Int, mpl: MultiPortalLayout): IntArray {
            return com.example.vision.ftcdepthai.openftc.easyopencv.OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                DEFAULT_VIEW_CONTAINER_ID, numPortals, mpl.viewportSplitMethod
            )
        }

        /**
         * Create a VisionPortal for an internal camera using default configuration parameters, and
         * skipping the use of the [Builder] pattern.
         * @param cameraDirection the internal camera to use
         * @param processors all the processors you want to inject into the portal
         * @return a configured, ready to use VisionPortal
         */
        @JvmStatic fun easyCreateWithDefaults(
            cameraDirection: BuiltinCameraDirection?,
            vararg processors: VisionProcessor?
        ): VisionPortal {
            return VisionPortal.Builder()
                .setCamera(cameraDirection)
                .addProcessors(*processors)
                .build()
        }

        /**
         * Create a VisionPortal for a webcam using default configuration parameters, and
         * skipping the use of the [Builder] pattern.
         * @param processors all the processors you want to inject into the portal
         * @return a configured, ready to use VisionPortal
         */
        @JvmStatic fun easyCreateWithDefaults(
            cameraName: CameraName?,
            vararg processors: VisionProcessor?
        ): VisionPortal {
            return VisionPortal.Builder()
                .setCamera(cameraName)
                .addProcessors(*processors)
                .build()
        }
    }
}