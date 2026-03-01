package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name="SimpleTeleWebcamAutoAim", group="TeleOp")
public class SimpleTelewAutoaim extends LinearOpMode {

    public boolean shooterOn = false;
    public boolean bPressedLastLoop = false;
    public boolean servosOn = false;
    public boolean autoAimOn = false;
    public boolean xPressedLastLoop = false;

    public DcMotor frontLeft, frontRight, backLeft, backRight, shooter;
    public CRServo speedServo1, torqueServo1, speedServo2, torqueServo2, torqueServo3, torqueServo4;

    // Vision
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // Hardware init
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        speedServo1 = hardwareMap.get(CRServo.class, "speedServo1");
        torqueServo1 = hardwareMap.get(CRServo.class, "torqueServo1");
        speedServo2 = hardwareMap.get(CRServo.class, "speedServo2");
        torqueServo2 = hardwareMap.get(CRServo.class, "torqueServo2");
        torqueServo3 = hardwareMap.get(CRServo.class, "torqueServo3");
        torqueServo4 = hardwareMap.get(CRServo.class, "torqueServo4");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Vision setup
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle shooter

            boolean bPressedNow = gamepad2.b;
            if (bPressedNow && !bPressedLastLoop) shooterOn = !shooterOn;
            bPressedLastLoop = bPressedNow;

            // Toggle servos
            boolean aPressedNow = gamepad2.a;
            if (aPressedNow) servosOn = !servosOn;

            // Toggle autoaim
            boolean xPressedNow = gamepad2.x;
            if (xPressedNow && !xPressedLastLoop) autoAimOn = !autoAimOn;
            xPressedLastLoop = xPressedNow;

            double flPower = 0, frPower = 0, blPower = 0, brPower = 0;

            if (autoAimOn) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                if (!detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0); // pick first tag
                    double errorX = tag.center.x - 320; // assuming 640px width
                    double errorY = tag.center.y - 240; // assuming 480px height

                    if (Math.abs(errorX) > 20) {
                        // Strafe left/right
                        double s = 0.2 * Math.signum(errorX);
                        flPower = +s; frPower = -s; blPower = -s; brPower = +s;
                        telemetry.addData("AutoAim", "Centering X");
                    } else if (Math.abs(errorY) > 20) {
                        // Drive forward/backward
                        double d = 0.2 * Math.signum(errorY);
                        flPower = d; frPower = d; blPower = d; brPower = d;
                        telemetry.addData("AutoAim", "Adjusting Y");
                    } else {
                        // Perfect position
                        flPower = frPower = blPower = brPower = 0;
                        telemetry.addData("AutoAim", "Autoaim initialized");
                    }
                } else {
                    // Search
                    flPower = -0.15; frPower = 0.15; blPower = -0.15; brPower = 0.15;
                    telemetry.addData("AutoAim", "Searching...");
                }
            } else {
                // Manual drive
                double drive = -gamepad2.left_stick_y;
                double strafe = gamepad2.left_stick_x;
                double rotate = gamepad2.right_stick_x;

                flPower = drive + strafe + rotate;
                frPower = drive - strafe - rotate;
                blPower = drive - strafe + rotate;
                brPower = drive + strafe - rotate;
            }

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            shooter.setPower(shooterOn ? 1.0 : 0.0);

            if (servosOn) {
                speedServo1.setPower(-1);
                torqueServo1.setPower(-1);
                torqueServo3.setPower(-1);
                speedServo2.setPower(1);
                torqueServo2.setPower(1);
                torqueServo4.setPower(1);
            } else {
                speedServo1.setPower(0);
                torqueServo1.setPower(0);
                torqueServo3.setPower(0);
                speedServo2.setPower(0);
                torqueServo2.setPower(0);
                torqueServo4.setPower(0);
            }

            telemetry.update();
        }
    }
}
