package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MyTargetProcessor implements VisionProcessor {

    // Color detection mats
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private volatile double targetX = 0.0;
    private volatile double targetY = 0.0;
    private volatile boolean targetFound = false;

    public int cameraWidth = 0;
    public int cameraHeight = 0;

    // Red HSV range
    private static final Scalar LOWER_BOUND = new Scalar(0, 100, 100);
    private static final Scalar UPPER_BOUND = new Scalar(10, 255, 255);

    // REAL AprilTag processor
    private AprilTagProcessor tagProcessor;

    private volatile int lastDetectedTagId = -1;

    // Motif
    private String[] motifOrder = {"green", "purple", "green"};


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraWidth = width;
        cameraHeight = height;

        // Fully configure the AprilTag processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        contours.clear();

        // ------------------------------
        // 1. Color detection
        // ------------------------------
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOWER_BOUND, UPPER_BOUND, mask);

        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint largest = contours.get(0);
            double maxArea = 0;

            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > maxArea) {
                    maxArea = area;
                    largest = c;
                }
            }

            if (maxArea > 100) {
                Rect box = Imgproc.boundingRect(largest);
                targetX = box.x + box.width / 2.0;
                targetY = box.y + box.height / 2.0;
                targetFound = true;
            } else {
                targetFound = false;
            }
        } else {
            targetFound = false;
        }


        // ------------------------------
        // 2. REAL AprilTag detection
        // MUST process frame or detections are junk
        // ------------------------------
        tagProcessor.processFrame(frame, captureTimeNanos);

        List<AprilTagDetection> detections = tagProcessor.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            lastDetectedTagId = tag.id;

            updateMotifOrder(lastDetectedTagId);
        }


        return frame;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if (targetFound) {
            Paint paint = new Paint();
            paint.setColor(Color.GREEN);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(5);

            float scaledX = (float) (targetX * scaleBmpPxToCanvasPx);
            float scaledY = (float) (targetY * scaleBmpPxToCanvasPx);

            canvas.drawCircle(scaledX, scaledY, 20, paint);
        }
    }


    // ------------------------------
    // Required getters
    // ------------------------------
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public boolean targetDetected() { return targetFound; }

    public int getDetectedTagId() { return lastDetectedTagId; }

    public String[] getMotifOrder() {
        return motifOrder;
    }


    // ------------------------------
    // Motif logic
    // ------------------------------
    public void updateMotifOrder(int tagId) {
        switch (tagId) {
            case 1:
                motifOrder = new String[]{"green", "purple", "purple"};
                break;
            case 2:
                motifOrder = new String[]{"purple", "green", "purple"};
                break;
            case 3:
                motifOrder = new String[]{"purple", "purple", "green"};
                break;
            default:
                motifOrder = new String[]{"green", "purple", "green"};
        }
    }
}
