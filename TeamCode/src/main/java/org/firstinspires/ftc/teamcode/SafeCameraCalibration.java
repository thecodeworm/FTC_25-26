package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

@TeleOp(name="SafeCameraCalibration", group="Testing")
public class SafeCameraCalibration extends LinearOpMode {

    OpenCvCamera webcam;

    // Camera intrinsics (manual calibration)
    double fx = 800;
    double fy = 800;
    double cx = 320;
    double cy = 240;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        // Stable pipeline that just copies frames safely
        webcam.setPipeline(new OpenCvPipeline() {
            Mat output = new Mat();

            @Override
            public Mat processFrame(Mat input) {
                // Make a safe copy of the frame
                input.copyTo(output);

                // Optional: convert to grayscale for easier visualization
                // Imgproc.cvtColor(output, output, Imgproc.COLOR_RGBA2GRAY);

                return output;  // Never return null
            }
        });

        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("FX", fx);
            telemetry.addData("FY", fy);
            telemetry.addData("CX", cx);
            telemetry.addData("CY", cy);
            telemetry.update();
        }
    }
}
