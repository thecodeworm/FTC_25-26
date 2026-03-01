package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayDeque;

@TeleOp(name = "HuskyLens Servo Turret (Position Mode)", group = "Vision")
public class Huskey extends LinearOpMode {

    // ===== HARDWARE =====
    HuskyLens husky;
    Servo turret;

    // ===== CAMERA CONSTANTS =====
    static final int IMAGE_WIDTH = 320;
    static final double CAMERA_FOV_DEG = 58.0; // adjust based on calibration

    // ===== TURRET SERVO CONSTANTS =====
    static final double TURRET_MIN = 0.17;     // physical min position
    static final double TURRET_MAX = 0.83;     // physical max position
    static final double TURRET_CENTER = 0.52;  // center for straight-ahead
    static final double TURRET_RANGE_DEG = 180.0;

    // ===== AIMING CONSTANTS =====
    static final double DEADBAND_DEG = 1.5;     // ignore small noise
    static final int SMOOTHING_SAMPLES = 5;    // rolling average

    ArrayDeque<Double> angleBuffer = new ArrayDeque<>();
    double lastServoPos = TURRET_CENTER;

    @Override
    public void runOpMode() {

        // ===== HARDWARE MAP =====
        husky = hardwareMap.get(HuskyLens.class, "huskylens");
        turret = hardwareMap.get(Servo.class, "turret");

        // ===== SERVO POSITION MODE =====
        turret.setPosition(TURRET_CENTER);  // Position Mode: absolute
        lastServoPos = TURRET_CENTER;

        // ===== HUSKYLENS SETUP =====
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("Servo Turret (Position Mode) Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // HuskyLens returns an array of blocks
            HuskyLens.Block[] blocks = husky.blocks();

            if (blocks != null && blocks.length > 0) {

                HuskyLens.Block target = blocks[0];  // pick first/primary tag

                // ===== ANGLE ERROR CALCULATION =====
                double xOffset = target.x - (IMAGE_WIDTH / 2.0);
                double angleError = (xOffset / IMAGE_WIDTH) * CAMERA_FOV_DEG;

                // ===== SMOOTHING =====
                angleBuffer.add(angleError);
                if (angleBuffer.size() > SMOOTHING_SAMPLES) {
                    angleBuffer.poll();
                }

                double avgAngle = 0;
                for (double a : angleBuffer) avgAngle += a;
                avgAngle /= angleBuffer.size();

                // ===== DEADZONE / DEAD-BAND =====
                if (Math.abs(avgAngle) > DEADBAND_DEG) {

                    // Convert angle to servo offset
                    double servoOffset = avgAngle / TURRET_RANGE_DEG;

                    double newServoPos = lastServoPos + servoOffset;

                    // Clamp to physical limits
                    newServoPos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, newServoPos));

                    // Set servo
                    turret.setPosition(newServoPos);
                    lastServoPos = newServoPos;
                }

                telemetry.addData("Tag ID", target.id);
                telemetry.addData("Angle Error (deg)", "%.2f", avgAngle);
                telemetry.addData("Servo Pos", "%.3f", lastServoPos);

            } else {
                // Target lost — hold last position
                telemetry.addLine("No Tag Detected — Holding Position");
                angleBuffer.clear();
            }

            telemetry.update();
            sleep(40);
        }
    }
}
