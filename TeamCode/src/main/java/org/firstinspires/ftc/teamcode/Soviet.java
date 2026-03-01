package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Soviet", group="TeleOp")
public class Soviet extends LinearOpMode {

    private boolean reverseOn = false;
    private boolean reverseYPressed = false;
    private boolean shooterOn = false;
    private boolean shooterBPressed = false;

    private boolean servosOn = false;
    private boolean servoAPressed = false;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // ---------------- MOTOR + SERVO MAP ----------------
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        CRServo speedServo1 = hardwareMap.get(CRServo.class, "speedServo1");
        CRServo torqueServo1 = hardwareMap.get(CRServo.class, "torqueServo1");
        CRServo speedServo2 = hardwareMap.get(CRServo.class, "speedServo2");
        CRServo torqueServo2 = hardwareMap.get(CRServo.class, "torqueServo2");
        CRServo torqueServo3 = hardwareMap.get(CRServo.class, "torqueServo3");
        CRServo torqueServo4 = hardwareMap.get(CRServo.class, "torqueServo4");
        CRServo revServo1 = hardwareMap.get(CRServo.class, "revServo1");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- APRILTAG INIT ----------------
        // Camera calibration intrinsics
        // Fx, Fy
        // Cx, Cy
        // K1,K2,K3
        // P1,P2
        // Skew
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                // Camera calibration intrinsics
                .setLensIntrinsics(
                        8514.65, // fx
                        8514.65, // fy
                        1017.54, // cx
                        817.294  // cy
                )
                .build();

        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                    .enableLiveView(true)
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addLine("Could not init webcam. Check name in config.");
            telemetry.addData("Exception", e.toString());
            telemetry.update();
        }

        telemetry.addLine("Initialized. Press Play.");
        telemetry.update();
        waitForStart();

        // Ensure streaming starts after play
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            visionPortal.resumeStreaming();
        }

        // ================= MAIN LOOP ==================
        while (opModeIsActive()) {

            // ---------------- DRIVETRAIN ----------------
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double rotate = gamepad2.right_stick_x;

            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            //Reverse Motors
            if(gamepad2.y){
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            };

            // Normalize powers
            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                    Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            // Apply base speed multiplier
            fl *= 0.7;
            fr *= 0.7;
            bl *= 0.7;
            br *= 0.7;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            //Reverse Toggle
            if (gamepad2.y && !shooterBPressed) reverseOn = !reverseOn;
            reverseYPressed = gamepad2.y;

            // ---------------- SHOOTER TOGGLE ----------------
            if (gamepad2.b && !shooterBPressed) shooterOn = !shooterOn;
            shooterBPressed = gamepad2.b;
            shooter.setPower(shooterOn ? 1 : 0);

            // ---------------- SERVO TOGGLE ----------------
            if (gamepad2.a && !servoAPressed) servosOn = !servosOn;
            servoAPressed = gamepad2.a;
            if (servosOn) {
                speedServo1.setPower(-1);
                torqueServo1.setPower(-1);
                torqueServo3.setPower(-1);
                speedServo2.setPower(1);
                torqueServo2.setPower(1);
                torqueServo4.setPower(1);
                revServo1.setPower(1);
            } else {
                speedServo1.setPower(0);
                torqueServo1.setPower(0);
                torqueServo3.setPower(0);
                speedServo2.setPower(0);
                torqueServo2.setPower(0);
                torqueServo4.setPower(0);
                revServo1.setPower(0);


            }

            // ----------- AUTO-ALIGN USING ftcPose -------------
            if (gamepad2.x && visionPortal != null) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
                    AprilTagDetection tag = detections.get(0);

                    double xError = tag.ftcPose.x;           // meters
                    double bearing = tag.ftcPose.bearing;    // degrees

                    double power = xError;             // forward/back
                    double turn = 0.05 * bearing;            // axial rotation

                    // Clamp values
                    power = Math.max(Math.min(power, 0.7), -0.7);
                    turn = Math.max(Math.min(turn, 0.5), -0.5);

                    // Apply mecanum drive logic
                    frontLeft.setPower(power + turn);
                    backLeft.setPower(power + turn);
                    frontRight.setPower(power - turn);
                    backRight.setPower(power - turn);

                    telemetry.addData("xError(m)", xError);
                    telemetry.addData("bearing(deg)", bearing);

                } else if (!detections.isEmpty() && detections.get(0).center != null) {
                    // fallback pixel-based alignment
                    AprilTagDetection tag = detections.get(0);
                    final int centerX = 400; // VisionPortal 800x448
                    double xErrorPx = centerX - tag.center.x;
                    double power = 0.002 * xErrorPx;
                    power = Math.max(Math.min(power, 0.5), -0.5);

                    frontLeft.setPower(power);
                    backLeft.setPower(-power);
                    frontRight.setPower(-power);
                    backRight.setPower(power);

                    telemetry.addData("xError(px)", xErrorPx);
                } else {
                    stopAll(frontLeft, frontRight, backLeft, backRight);
                    telemetry.addLine("No tag detected");
                }
            }

            telemetry.addData("Camera State", visionPortal != null ? visionPortal.getCameraState() : "null");
            telemetry.update();
        }
    }

    private void stopAll(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}