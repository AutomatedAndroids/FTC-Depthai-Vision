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
//import com.example.ftc_depth_ai.ftc.VisionProcessor
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition
//import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters
//@Deprecated("This class has been disabled to protect from internal errors, such as classpath. please use the traditional system.")
//abstract class TfodProcessor : VisionProcessor {
//    class Builder {
//        private val builder = TfodParameters.Builder()
//
//        /**
//         * Set the name of the asset where the model is found.
//         */
//        fun setModelAssetName(assetName: String?): Builder {
//            builder.setModelAssetName(assetName)
//            return this
//        }
//
//        /**
//         * Set the name of the file where the model is found.
//         */
//        fun setModelFileName(fileName: String?): Builder {
//            builder.setModelFileName(fileName)
//            return this
//        }
//
//        /**
//         * Set the full ordered list of labels the model is trained to recognize.
//         */
//        fun setModelLabels(labels: List<String?>?): Builder {
//            builder.setModelLabels(labels)
//            return this
//        }
//
//        fun setModelLabels(labels: Array<String?>?): Builder {
//            builder.setModelLabels(labels)
//            return this
//        }
//
//        /**
//         * Set whether the model is a TensorFlow2 model.
//         */
//        fun setIsModelTensorFlow2(isModelTensorFlow2: Boolean): Builder {
//            builder.setIsModelTensorFlow2(isModelTensorFlow2)
//            return this
//        }
//
//        /**
//         * Set whether the model is quantized.
//         */
//        fun setIsModelQuantized(isModelQuantized: Boolean): Builder {
//            builder.setIsModelQuantized(isModelQuantized)
//            return this
//        }
//
//        /**
//         * Set the size, in pixels, of images input to the network.
//         */
//        fun setModelInputSize(inputSize: Int): Builder {
//            builder.setModelInputSize(inputSize)
//            return this
//        }
//
//        /**
//         * Set the aspect ratio for the images used when the model was created.
//         */
//        fun setModelAspectRatio(modelAspectRatio: Double): Builder {
//            builder.setModelAspectRatio(modelAspectRatio)
//            return this
//        }
//
//        /**
//         * Set the number of executor threads to use. Each executor corresponds to one TensorFlow
//         * Object Detector.
//         */
//        fun setNumExecutorThreads(numExecutorThreads: Int): Builder {
//            builder.setNumExecutorThreads(numExecutorThreads)
//            return this
//        }
//
//        /**
//         * Set the number of threads to allow each individual TensorFlow object detector to use.
//         */
//        fun setNumDetectorThreads(numDetectorThreads: Int): Builder {
//            builder.setNumDetectorThreads(numDetectorThreads)
//            return this
//        }
//
//        /**
//         * Set the maximum number of recognitions the network will return.
//         */
//        fun setMaxNumRecognitions(maxNumRecognitions: Int): Builder {
//            builder.setMaxNumRecognitions(maxNumRecognitions)
//            return this
//        }
//
//        /**
//         * Set whether to use the tracker.
//         */
//        fun setUseObjectTracker(useObjectTracker: Boolean): Builder {
//            builder.setUseObjectTracker(useObjectTracker)
//            return this
//        }
//
//        /**
//         * Set the maximum percentage of a box that can be overlapped by another box at recognition time.
//         */
//        fun setTrackerMaxOverlap(trackerMaxOverlap: Float): Builder {
//            builder.setTrackerMaxOverlap(trackerMaxOverlap)
//            return this
//        }
//
//        /**
//         * Set the minimum size of an object that the tracker will track.
//         */
//        fun setTrackerMinSize(trackerMinSize: Float): Builder {
//            builder.setTrackerMinSize(trackerMinSize)
//            return this
//        }
//
//        /**
//         * Allow replacement of the tracked box with new results if correlation has dropped below
//         * trackerMarginalCorrelation.
//         */
//        fun setTrackerMarginalCorrelation(trackerMarginalCorrelation: Float): Builder {
//            builder.setTrackerMarginalCorrelation(trackerMarginalCorrelation)
//            return this
//        }
//
//        /**
//         * Consider an object to be lost if correlation falls below trackerMinCorrelation.
//         */
//        fun setTrackerMinCorrelation(trackerMinCorrelation: Float): Builder {
//            builder.setTrackerMinCorrelation(trackerMinCorrelation)
//            return this
//        }
//
//        /**
//         * Returns a TfodProcessor object.
//         */
//        fun build(): TfodProcessor {
//            return TfodProcessorImpl(builder.build())
//        }
//    }
//
//    /**
//     * Set the minimum confidence at which to keep recognitions.
//     */
//    abstract fun setMinResultConfidence(minResultConfidence: Float)
//
//    /**
//     * Sets the number of pixels to obscure on the left, top, right, and bottom edges of each image
//     * passed to the TensorFlow object detector. The size of the images are not changed, but the
//     * pixels in the margins are colored black.
//     */
//    abstract fun setClippingMargins(left: Int, top: Int, right: Int, bottom: Int)
//
//    /**
//     * Indicates that only the zoomed center area of each image will be passed to the TensorFlow
//     * object detector. For no zooming, set magnification to 1.0.
//     */
//    abstract fun setZoom(magnification: Double)
//
//    /**
//     * Gets a list containing the latest recognitions, which may be stale.
//     */
//    abstract val recognitions: List<Recognition?>?
//
//    /**
//     * Gets a list containing recognitions that were detected since the last call to this method,
//     * or null if no new recognitions are available. This is useful to avoid re-processing the same
//     * recognitions multiple times.
//     * @return a list containing fresh recognitions, or null.
//     */
//    abstract val freshRecognitions: List<Recognition?>?
//
//    /**
//     * Perform whatever cleanup is necessary to release all acquired resources.
//     */
//    abstract fun shutdown()
//
//    companion object {
//        fun easyCreateWithDefaults(): TfodProcessor {
//            return Builder().build()
//        }
//    }
//}