package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.MyTargetProcessor;

@TeleOp(name="MecanumAimer", group="Utility")
public class MecanumAimer extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    private VisionPortal visionPortal;
    private MyTargetProcessor targetProcessor;

    private static final double CAMERA_HFOV_DEG = 60.0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        targetProcessor = new MyTargetProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(targetProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {


            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            frontRight.setPower(y - x - rx);
            backLeft.setPower(y - x + rx);
            backRight.setPower(y + x - rx);

            if (targetProcessor.targetDetected()) {

                double px = targetProcessor.getTargetX();
                int width = targetProcessor.cameraWidth;

                double anglePerPixel = CAMERA_HFOV_DEG / width;
                double center = width / 2.0;

                double angleError = (px - center) * anglePerPixel;

                telemetry.addLine("=== AprilTag Found ===");
                telemetry.addData("Turn Angle (deg)", angleError);
                telemetry.addLine(angleError > 0 ?
                        "Turn RIGHT to align" :
                        "Turn LEFT to align");

            } else {
                telemetry.addLine("No AprilTag detected");
            }

            telemetry.update();
        }
    }
}