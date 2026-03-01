package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ShooterAutoAimVelocityControlled")
public class ShooterAutoAimVelocityControlled extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx shooterMotor;

    // --- Shooter Control ---
    private double targetPower = 0.0;
    private double targetVelocity = 0.0;
    private boolean shootRequest = false;
    private boolean shotReady = false;
    private double velocityMargin = 0.10; // ±10% margin

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Init Hardware ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            // ------------------------------------------------------
            // 1. AUTO AIM LOGIC DETERMINES TARGET POWER (REPLACE THIS)
            // ------------------------------------------------------
            // Example: mapping trigger directly to power (replace with your math)
            targetPower = gamepad1.right_trigger; // 0 to 1.0

            // Safety clamp
            targetPower = Math.max(0, Math.min(1.0, targetPower));

            // ------------------------------------------------------
            // 2. CONVERT POWER → TARGET VELOCITY
            //    (YOU MUST ADJUST maxRPM TO MATCH YOUR MOTOR!)
            // ------------------------------------------------------
            double maxRPM = 6000; // <-- CHANGE FOR YOUR MOTOR!!!
            double ticksPerRev = 28; // GoBILDA 5202 series
            double maxTicksPerSecond = (maxRPM / 60.0) * ticksPerRev;
            targetVelocity = targetPower * maxTicksPerSecond;

            // ------------------------------------------------------
            // 3. COMMAND VELOCITY USING BUILT-IN PID
            // ------------------------------------------------------
            shooterMotor.setVelocity(targetVelocity);

            // ------------------------------------------------------
            // 4. CHECK IF VELOCITY IS WITHIN MARGIN
            // ------------------------------------------------------
            double currentVelocity = shooterMotor.getVelocity(); // ticks/sec

            if (targetVelocity > 0) {
                double velocityError = Math.abs(currentVelocity - targetVelocity) / targetVelocity;

                if (velocityError <= velocityMargin) {
                    shotReady = true;
                } else {
                    shotReady = false;
                }
            } else {
                shotReady = false;
            }

            // ------------------------------------------------------
            // 5. SHOOT CONTROL (BETWEEN SHOTS CHECKS READY STATE)
            // ------------------------------------------------------
            if (gamepad1.right_bumper) {
                shootRequest = true;
            }

            if (shootRequest && shotReady) {
                // >>> FIRE SERVO OR INDEXER HERE <<<
                // Example:
                // indexerServo.setPosition(FIRE_POS);
                timer.reset();
                while (timer.milliseconds() < 150 && opModeIsActive()) {
                    // wait for shot
                }
                // indexerServo.setPosition(REST_POS);

                shootRequest = false; // done
            }

            // ------------------------------------------------------
            // Telemetry
            // ------------------------------------------------------
            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Target Velocity", "%.0f", targetVelocity);
            telemetry.addData("Current Velocity", "%.0f", currentVelocity);
            telemetry.addData("Velocity Error", "%.2f%%", targetVelocity > 0 ? (100* Math.abs(currentVelocity-targetVelocity)/targetVelocity) : 0);
            telemetry.addData("Shot Ready?", shotReady);
            telemetry.addData("Shoot Request?", shootRequest);
            telemetry.update();
        }
    }
}
