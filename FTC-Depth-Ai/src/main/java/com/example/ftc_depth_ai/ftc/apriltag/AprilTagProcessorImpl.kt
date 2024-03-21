package com.example.ftc_depth_ai.ftc.apriltag

import android.graphics.Canvas
import android.util.Log
import com.qualcomm.robotcore.util.MovingStatistics
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3
import org.opencv.imgproc.Imgproc
import com.example.vision.ftcdepthai.openftc.apriltag.AprilTagDetectorJNI
import com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI
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
class AprilTagProcessorImpl(
    private var fx: Double,
    private var fy: Double,
    private var cx: Double,
    private var cy: Double,
    private val outputUnitsLength: DistanceUnit,
    private val outputUnitsAngle: AngleUnit,
    private val tagLibrary: AprilTagLibrary,
    private val drawAxes: Boolean,
    private val drawCube: Boolean,
    private val drawOutline: Boolean,
    private val drawTagID: Boolean,
    tagFamily: TagFamily,
    threads: Int
) : AprilTagProcessor() {
    private var nativeApriltagPtr: Long
    private val grey = Mat()
    override var detections = ArrayList<AprilTagDetection>()
        private set
    private var detectionsUpdate: ArrayList<AprilTagDetection>? = ArrayList()
    private val detectionsUpdateSync = Any()
    private var cameraMatrix: Mat? = null
    private var decimation = 0f
    private var needToSetDecimation = false
    private val decimationSync = Any()
    private var canvasAnnotator: AprilTagCanvasAnnotator? = null

    @Volatile
    private var poseSolver = PoseSolver.OPENCV_ITERATIVE
    protected fun finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0L) {
            // Delete the native context we created in the constructor
            com.example.vision.ftcdepthai.openftc.apriltag.AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        } else {
            println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL")
        }
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // If the user didn't give us a calibration, but we have one built in,
        // then go ahead and use it!!
        if (calibration != null && fx == 0.0 && fy == 0.0 && cx == 0.0 && cy == 0.0 && !(calibration.focalLengthX == 0f && calibration.focalLengthY == 0f && calibration.principalPointX == 0f && calibration.principalPointY == 0f)) // needed because we may get an all zero calibration to indicate none, instead of null
        {
            fx = calibration.focalLengthX.toDouble()
            fy = calibration.focalLengthY.toDouble()
            cx = calibration.principalPointX.toDouble()
            cy = calibration.principalPointY.toDouble()
            Log.d(
                TAG, String.format(
                    "User did not provide a camera calibration; but we DO have a built in calibration we can use.\n [%dx%d] (may be scaled) %s\nfx=%7.3f fy=%7.3f cx=%7.3f cy=%7.3f",
                    calibration.size.width,
                    calibration.size.height,
                    calibration.identity.toString(),
                    fx,
                    fy,
                    cx,
                    cy
                )
            )
        } else if (fx == 0.0 && fy == 0.0 && cx == 0.0 && cy == 0.0) {
            // set it to *something* so we don't crash the native code
            val warning =
                "User did not provide a camera calibration, nor was a built-in calibration found for this camera; 6DOF pose data will likely be inaccurate."
            Log.d(TAG, warning)
            RobotLog.addGlobalWarningMessage(warning)
            fx = 578.272
            fy = 578.272
            cx = (width / 2).toDouble()
            cy = (height / 2).toDouble()
        } else {
            Log.d(
                TAG, String.format(
                    "User provided their own camera calibration fx=%7.3f fy=%7.3f cx=%7.3f cy=%7.3f",
                    fx, fy, cx, cy
                )
            )
        }
        constructMatrix()
        canvasAnnotator = AprilTagCanvasAnnotator(cameraMatrix)
    }

    override fun processFrame(input: Mat?, captureTimeNanos: Long): Any? {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY)
        synchronized(decimationSync) {
            if (needToSetDecimation) {
                com.example.vision.ftcdepthai.openftc.apriltag.AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation)
                needToSetDecimation = false
            }
        }

        // Run AprilTag
        detections = runAprilTagDetectorForMultipleTagSizes(captureTimeNanos)
        synchronized(detectionsUpdateSync) { detectionsUpdate = detections }

        // TODO do we need to deep copy this so the user can't mess with it before use in onDrawFrame()?
        return detections
    }

    private val solveTime = MovingStatistics(50)

    // We cannot use runAprilTagDetectorSimple because we cannot assume tags are all the same size
    fun runAprilTagDetectorForMultipleTagSizes(captureTimeNanos: Long): ArrayList<AprilTagDetection> {
        val ptrDetectionArray = com.example.vision.ftcdepthai.openftc.apriltag.AprilTagDetectorJNI.runApriltagDetector(
            nativeApriltagPtr,
            grey.dataAddr(),
            grey.width(),
            grey.height()
        )
        if (ptrDetectionArray != 0L) {
            val detectionPointers = com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getDetectionPointers(ptrDetectionArray)
            val detections = ArrayList<AprilTagDetection>(detectionPointers.size)
            for (ptrDetection in detectionPointers) {
                val metadata = tagLibrary.lookupTag(com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getId(ptrDetection))
                val corners = com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getCorners(ptrDetection)
                val cornerPts = arrayOfNulls<Point>(4)
                for (p in 0..3) {
                    cornerPts[p] = Point(corners[p][0], corners[p][1])
                }
                var rawPose: AprilTagPoseRaw?
                var ftcPose: AprilTagPoseFtc?
                if (metadata != null) {
                    val solver = poseSolver // snapshot, can change
                    val startSolveTime = System.currentTimeMillis()
                    if (solver == PoseSolver.APRILTAG_BUILTIN) {
                        val pose = com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getPoseEstimate(
                            ptrDetection,
                            outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                            fx, fy, cx, cy
                        )

                        // Build rotation matrix
                        val rotMtxVals = FloatArray(3 * 3)
                        for (i in 0..8) {
                            rotMtxVals[i] = pose[3 + i].toFloat()
                        }
                        rawPose = AprilTagPoseRaw(
                            pose[0], pose[1], pose[2],  // x y z
                            GeneralMatrixF(3, 3, rotMtxVals)
                        ) // R
                    } else {
                        val opencvPose = poseFromTrapezoid(
                            cornerPts,
                            cameraMatrix,
                            outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                            solver.code
                        )

                        // Build rotation matrix
                        val R = Mat(3, 3, CvType.CV_32F)
                        Calib3d.Rodrigues(opencvPose.rvec, R)
                        val tmp2 = FloatArray(9)
                        R[0, 0, tmp2]
                        rawPose = AprilTagPoseRaw(
                            opencvPose.tvec[0, 0][0],  // x
                            opencvPose.tvec[1, 0][0],  // y
                            opencvPose.tvec[2, 0][0],  // z
                            GeneralMatrixF(3, 3, tmp2)
                        ) // R
                    }
                    val endSolveTime = System.currentTimeMillis()
                    solveTime.add((endSolveTime - startSolveTime).toDouble())
                } else {
                    // We don't know anything about the tag size so we can't solve the pose
                    rawPose = null
                }
                ftcPose = if (rawPose != null) {
                    val rot = Orientation.getOrientation(
                        rawPose.R,
                        AxesReference.INTRINSIC,
                        AxesOrder.YXZ,
                        outputUnitsAngle
                    )
                    AprilTagPoseFtc(
                        rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                        rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                        -rawPose.y,  // z       system and the FTC coordinate system
                        -rot.firstAngle.toDouble(),  // yaw
                        rot.secondAngle.toDouble(),  // pitch
                        rot.thirdAngle.toDouble(),  // roll
                        Math.hypot(rawPose.x, rawPose.z),  // range
                        outputUnitsAngle.fromUnit(
                            AngleUnit.RADIANS,
                            Math.atan2(-rawPose.x, rawPose.z)
                        ),  // bearing
                        outputUnitsAngle.fromUnit(
                            AngleUnit.RADIANS,
                            Math.atan2(-rawPose.y, rawPose.z)
                        )
                    ) // elevation
                } else {
                    null
                }
                val center = com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getCenterpoint(ptrDetection)
                detections.add(
                    AprilTagDetection(
                        com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getId(ptrDetection),
                        com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getHamming(ptrDetection),
                        com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.getDecisionMargin(ptrDetection),
                        Point(center[0], center[1]),
                        cornerPts,
                        metadata,
                        ftcPose,
                        rawPose,
                        captureTimeNanos
                    )
                )
            }
            com.example.vision.ftcdepthai.openftc.apriltag.ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray)
            return detections
        }
        return ArrayList()
    }

    private val drawSync = Any()

    init {

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr =
            com.example.vision.ftcdepthai.openftc.apriltag.AprilTagDetectorJNI.createApriltagDetector(tagFamily.ATLibTF.string, 3f, threads)
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        // Only one draw operation at a time thank you very much.
        // (we could be called from two different threads - viewport or camera stream)
        synchronized(drawSync) {
            if ((drawAxes || drawCube || drawOutline || drawTagID) && userContext != null) {
                canvasAnnotator!!.noteDrawParams(scaleBmpPxToCanvasPx, scaleCanvasDensity)
                val dets = userContext as ArrayList<AprilTagDetection>

                // For fun, draw 6DOF markers on the image.
                for (detection in dets) {
                    if (drawTagID) {
                        canvasAnnotator!!.drawTagID(detection, canvas)
                    }

                    // Could be null if we couldn't solve the pose earlier due to not knowing tag size
                    if (detection.rawPose != null) {
                        val metadata = tagLibrary.lookupTag(detection.id)
                        val tagSize =
                            outputUnitsLength.fromUnit(metadata!!.distanceUnit, metadata.tagsize)
                        if (drawOutline) {
                            canvasAnnotator!!.drawOutlineMarker(detection, canvas, tagSize)
                        }
                        if (drawAxes) {
                            canvasAnnotator!!.drawAxisMarker(detection, canvas, tagSize)
                        }
                        if (drawCube) {
                            canvasAnnotator!!.draw3dCubeMarker(detection, canvas, tagSize)
                        }
                    }
                }
            }
        }
    }

    override fun setDecimation(decimation: Float) {
        synchronized(decimationSync) {
            this.decimation = decimation
            needToSetDecimation = true
        }
    }

    override fun setPoseSolver(poseSolver: PoseSolver) {
        this.poseSolver = poseSolver
    }

    override val perTagAvgPoseSolveTime: Int
        get() = Math.round(solveTime.mean).toInt()
    override val freshDetections: ArrayList<AprilTagDetection>?
        get() {
            synchronized(detectionsUpdateSync) {
                val ret = detectionsUpdate
                detectionsUpdate = null
                return ret
            }
        }

    fun constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        cameraMatrix!!.put(0, 0, fx)
        cameraMatrix!!.put(0, 1, 0.0)
        cameraMatrix!!.put(0, 2, cx)
        cameraMatrix!!.put(1, 0, 0.0)
        cameraMatrix!!.put(1, 1, fy)
        cameraMatrix!!.put(1, 2, cy)
        cameraMatrix!!.put(2, 0, 0.0)
        cameraMatrix!!.put(2, 1, 0.0)
        cameraMatrix!!.put(2, 2, 1.0)
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose {
        var rvec: Mat
        var tvec: Mat

        constructor() {
            rvec = Mat(3, 1, CvType.CV_32F)
            tvec = Mat(3, 1, CvType.CV_32F)
        }

        constructor(rvec: Mat, tvec: Mat) {
            this.rvec = rvec
            this.tvec = tvec
        }
    }

    companion object {
        const val TAG = "AprilTagProcessorImpl"

        /**
         * Converts an AprilTag pose to an OpenCV pose
         * @param aprilTagPose pose to convert
         * @return OpenCV output pose
         */
        fun aprilTagPoseToOpenCvPose(aprilTagPose: AprilTagPoseRaw?): Pose {
            val pose = Pose()
            pose.tvec.put(0, 0, aprilTagPose!!.x)
            pose.tvec.put(1, 0, aprilTagPose.y)
            pose.tvec.put(2, 0, aprilTagPose.z)
            val R = Mat(3, 3, CvType.CV_32F)
            for (i in 0..2) {
                for (j in 0..2) {
                    R.put(i, j, aprilTagPose.R[i, j].toDouble())
                }
            }
            Calib3d.Rodrigues(R, pose.rvec)
            return pose
        }

        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsize the original length of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        fun poseFromTrapezoid(
            points: Array<Point?>,
            cameraMatrix: Mat?,
            tagsize: Double,
            solveMethod: Int
        ): Pose {
            // The actual 2d points of the tag detected in the image
            val points2d = MatOfPoint2f(*points)

            // The 3d points of the tag in an 'ideal projection'
            val arrayPoints3d = arrayOfNulls<Point3>(4)
            arrayPoints3d[0] = Point3(-tagsize / 2, tagsize / 2, 0.0)
            arrayPoints3d[1] = Point3(tagsize / 2, tagsize / 2, 0.0)
            arrayPoints3d[2] = Point3(tagsize / 2, -tagsize / 2, 0.0)
            arrayPoints3d[3] = Point3(-tagsize / 2, -tagsize / 2, 0.0)
            val points3d = MatOfPoint3f(*arrayPoints3d)

            // Using this information, actually solve for pose
            val pose = Pose()
            Calib3d.solvePnP(
                points3d,
                points2d,
                cameraMatrix,
                MatOfDouble(),
                pose.rvec,
                pose.tvec,
                false,
                solveMethod
            )
            return pose
        }
    }
}