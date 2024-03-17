package com.example.ftc_depth_ai.ftc.apriltag

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Typeface
import com.example.ftc_depth_ai.ftc.apriltag.AprilTagProcessorImpl.Pose
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point3

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
class AprilTagCanvasAnnotator(val cameraMatrix: Mat?) {
    var bmpPxToCanvasPx = 0f
    var canvasDensityScale = 0f
    var redAxisPaint = LinePaint(Color.RED)
    var greenAxisPaint = LinePaint(Color.GREEN)
    var blueAxisPaint = LinePaint(Color.BLUE)
    var boxPillarPaint = LinePaint(Color.rgb(7, 197, 235))
    var boxTopPaint = LinePaint(Color.GREEN)

    class LinePaint(color: Int) : Paint() {
        init {
            setColor(color)
            isAntiAlias = true
            strokeCap = Cap.ROUND
        }
    }

    var textPaint: Paint
    var rectPaint: Paint

    init {
        textPaint = Paint()
        textPaint.color = Color.WHITE
        textPaint.isAntiAlias = true
        textPaint.setTypeface(Typeface.DEFAULT_BOLD)
        rectPaint = Paint()
        rectPaint.isAntiAlias = true
        rectPaint.color = Color.rgb(12, 145, 201)
        rectPaint.style = Paint.Style.FILL
    }

    fun noteDrawParams(bmpPxToCanvasPx: Float, canvasDensityScale: Float) {
        if (bmpPxToCanvasPx != this.bmpPxToCanvasPx || canvasDensityScale != this.canvasDensityScale) {
            this.bmpPxToCanvasPx = bmpPxToCanvasPx
            this.canvasDensityScale = canvasDensityScale
            textPaint.textSize = 40 * canvasDensityScale
        }
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     * @param tagsize size of the tag in SAME UNITS as pose
     */
    fun drawAxisMarker(detection: AprilTagDetection, canvas: Canvas?, tagsize: Double) {
        //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsize, tagsize);
        val pose: Pose = AprilTagProcessorImpl.Companion.aprilTagPoseToOpenCvPose(detection.rawPose)

        // in meters, actually.... will be mapped to screen coords
        val axisLength = (tagsize / 2.0).toFloat()

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
            Point3(0.0, 0.0, 0.0),
            Point3(-axisLength.toDouble(), 0.0, 0.0),
            Point3(0.0, -axisLength.toDouble(), 0.0),
            Point3(0.0, 0.0, -axisLength.toDouble())
        )

        // Project those points onto the image
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(
            axis,
            pose.rvec,
            pose.tvec,
            cameraMatrix,
            MatOfDouble(),
            matProjectedPoints
        )
        val projectedPoints = matProjectedPoints.toArray()

        // The projection we did was good for the original resolution image, but now
        // we need to scale those to their locations on the canvas.
        for (p in projectedPoints) {
            p.x *= bmpPxToCanvasPx.toDouble()
            p.y *= bmpPxToCanvasPx.toDouble()
        }

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        val dist3d = Math.sqrt(
            Math.pow(detection.rawPose!!.x, 2.0) + Math.pow(
                detection.rawPose.y, 2.0
            ) + Math.pow(detection.rawPose.z, 2.0)
        )
        val axisThickness =
            (5 / dist3d * (tagsize / 0.166) * bmpPxToCanvasPx).toFloat() // looks about right I guess
        redAxisPaint.strokeWidth = axisThickness
        greenAxisPaint.strokeWidth = axisThickness
        blueAxisPaint.strokeWidth = axisThickness

        // Now draw the axes
        canvas!!.drawLine(
            projectedPoints[0].x.toFloat(),
            projectedPoints[0].y.toFloat(),
            projectedPoints[1].x.toFloat(),
            projectedPoints[1].y.toFloat(),
            redAxisPaint
        )
        canvas.drawLine(
            projectedPoints[0].x.toFloat(),
            projectedPoints[0].y.toFloat(),
            projectedPoints[2].x.toFloat(),
            projectedPoints[2].y.toFloat(),
            greenAxisPaint
        )
        canvas.drawLine(
            projectedPoints[0].x.toFloat(),
            projectedPoints[0].y.toFloat(),
            projectedPoints[3].x.toFloat(),
            projectedPoints[3].y.toFloat(),
            blueAxisPaint
        )
    }

    /**
     * Draw a 3D cube marker on a detection
     *
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     * @param tagsize size of the tag in SAME UNITS as pose
     */
    fun draw3dCubeMarker(detection: AprilTagDetection, canvas: Canvas?, tagsize: Double) {
        //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsize, tagsize);
        val pose: Pose = AprilTagProcessorImpl.Companion.aprilTagPoseToOpenCvPose(detection.rawPose)

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
            Point3(-tagsize / 2, tagsize / 2, 0.0),
            Point3(tagsize / 2, tagsize / 2, 0.0),
            Point3(tagsize / 2, -tagsize / 2, 0.0),
            Point3(-tagsize / 2, -tagsize / 2, 0.0),
            Point3(-tagsize / 2, tagsize / 2, -tagsize),
            Point3(tagsize / 2, tagsize / 2, -tagsize),
            Point3(tagsize / 2, -tagsize / 2, -tagsize),
            Point3(-tagsize / 2, -tagsize / 2, -tagsize)
        )

        // Project those points
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(
            axis,
            pose.rvec,
            pose.tvec,
            cameraMatrix,
            MatOfDouble(),
            matProjectedPoints
        )
        val projectedPoints = matProjectedPoints.toArray()

        // The projection we did was good for the original resolution image, but now
        // we need to scale those to their locations on the canvas.
        for (p in projectedPoints) {
            p.x *= bmpPxToCanvasPx.toDouble()
            p.y *= bmpPxToCanvasPx.toDouble()
        }

        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        val dist3d = Math.sqrt(
            Math.pow(detection.rawPose!!.x, 2.0) + Math.pow(
                detection.rawPose.y, 2.0
            ) + Math.pow(detection.rawPose.z, 2.0)
        )
        val thickness =
            (3.5 / dist3d * (tagsize / 0.166) * bmpPxToCanvasPx).toFloat() // looks about right I guess
        boxPillarPaint.strokeWidth = thickness
        boxTopPaint.strokeWidth = thickness
        val pillarPts = FloatArray(16)

        // Pillars
        for (i in 0..3) {
            pillarPts[i * 4 + 0] = projectedPoints[i].x.toFloat()
            pillarPts[i * 4 + 1] = projectedPoints[i].y.toFloat()
            pillarPts[i * 4 + 2] = projectedPoints[i + 4].x.toFloat()
            pillarPts[i * 4 + 3] = projectedPoints[i + 4].y.toFloat()
        }
        canvas!!.drawLines(pillarPts, boxPillarPaint)

        // Top lines
        val topPts = floatArrayOf(
            projectedPoints[4].x.toFloat(),
            projectedPoints[4].y.toFloat(),
            projectedPoints[5].x.toFloat(),
            projectedPoints[5].y.toFloat(),
            projectedPoints[5].x.toFloat(),
            projectedPoints[5].y.toFloat(),
            projectedPoints[6].x.toFloat(),
            projectedPoints[6].y.toFloat(),
            projectedPoints[6].x.toFloat(),
            projectedPoints[6].y.toFloat(),
            projectedPoints[7].x.toFloat(),
            projectedPoints[7].y.toFloat(),
            projectedPoints[4].x.toFloat(),
            projectedPoints[4].y.toFloat(),
            projectedPoints[7].x.toFloat(),
            projectedPoints[7].y.toFloat()
        )
        canvas.drawLines(topPts, boxTopPaint)
    }

    /**
     * Draw an outline marker on the detection
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     * @param tagsize size of the tag in SAME UNITS as pose
     */
    fun drawOutlineMarker(detection: AprilTagDetection, canvas: Canvas?, tagsize: Double) {
        // Use the 3D distance to the target, as well as the physical size of the
        // target in the real world to scale the thickness of lines.
        val dist3d = Math.sqrt(
            Math.pow(detection.rawPose!!.x, 2.0) + Math.pow(
                detection.rawPose.y, 2.0
            ) + Math.pow(detection.rawPose.z, 2.0)
        )
        val axisThickness =
            (5 / dist3d * (tagsize / 0.166) * bmpPxToCanvasPx).toFloat() // looks about right I guess
        redAxisPaint.strokeWidth = axisThickness
        greenAxisPaint.strokeWidth = axisThickness
        blueAxisPaint.strokeWidth = axisThickness
        canvas!!.drawLine(
            detection.corners[0]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[0]!!.y.toFloat() * bmpPxToCanvasPx,
            detection.corners[1]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[1]!!.y.toFloat() * bmpPxToCanvasPx,
            redAxisPaint
        )
        canvas.drawLine(
            detection.corners[1]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[1]!!.y.toFloat() * bmpPxToCanvasPx,
            detection.corners[2]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[2]!!.y.toFloat() * bmpPxToCanvasPx,
            greenAxisPaint
        )
        canvas.drawLine(
            detection.corners[0]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[0]!!.y.toFloat() * bmpPxToCanvasPx,
            detection.corners[3]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[3]!!.y.toFloat() * bmpPxToCanvasPx,
            blueAxisPaint
        )
        canvas.drawLine(
            detection.corners[2]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[2]!!.y.toFloat() * bmpPxToCanvasPx,
            detection.corners[3]!!.x.toFloat() * bmpPxToCanvasPx,
            detection.corners[3]!!.y.toFloat() * bmpPxToCanvasPx,
            blueAxisPaint
        )
    }

    /**
     * Draw the Tag's ID on the tag
     * @param detection the detection to draw
     * @param canvas the canvas to draw on
     */
    fun drawTagID(detection: AprilTagDetection, canvas: Canvas?) {
        val cornerRound = 5 * canvasDensityScale
        val tag_id_width = 140 * canvasDensityScale
        val tag_id_height = 50 * canvasDensityScale
        val id_x = detection.center.x.toFloat() * bmpPxToCanvasPx - tag_id_width / 2
        val id_y = detection.center.y.toFloat() * bmpPxToCanvasPx - tag_id_height / 2
        val tag_id_text_x = id_x + 10 * canvasDensityScale
        val tag_id_text_y = id_y + 40 * canvasDensityScale
        val lowerLeft = detection.corners[0]
        val lowerRight = detection.corners[1]
        canvas!!.save()
        canvas.rotate(
            Math.toDegrees(
                Math.atan2(
                    lowerRight!!.y - lowerLeft!!.y,
                    lowerRight.x - lowerLeft.x
                )
            ).toFloat(),
            detection.center.x.toFloat() * bmpPxToCanvasPx,
            detection.center.y.toFloat() * bmpPxToCanvasPx
        )
        canvas.drawRoundRect(
            id_x,
            id_y,
            id_x + tag_id_width,
            id_y + tag_id_height,
            cornerRound,
            cornerRound,
            rectPaint
        )
        canvas.drawText(
            String.format("ID %03d", detection.id),
            tag_id_text_x,
            tag_id_text_y,
            textPaint
        )
        canvas.restore()
    }
}