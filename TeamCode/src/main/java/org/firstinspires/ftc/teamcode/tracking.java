package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "HuskyLens Turret Tracker", group = "Vision")
public class tracking extends LinearOpMode {

    HuskyLens husky;
    DcMotorEx turretMotor;

    // ═══════════════════════════════════════════════════════════════════
    // MOTOR CONFIGURATION - GoBILDA 312 RPM (5203-2403)
    // ═══════════════════════════════════════════════════════════════════

    final double MOTOR_TICKS_PER_REV = 537.7;  // GoBILDA 312 RPM motor
    final double GEAR_RATIO = 1.0;              // Set to your external gearing
    final double TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    // Result: 1.49 ticks per degree (good resolution!)

    // ═══════════════════════════════════════════════════════════════════
    // PID TUNING - MAXIMUM SPEED WITH PRECISION
    // ═══════════════════════════════════════════════════════════════════

    final double KP = 0.035;         // AGGRESSIVE - instant response
    final double KI = 0.00025;       // Higher - quick final settling
    final double KD = 0.0035;        // STRONG damping - prevents overshoot
    final double KF = 0.0;           // Not needed

    final double MAX_POWER = 0.65;   // FAST - as high as belt allows
    final double MIN_POWER = 0.18;   // Strong enough to move instantly
    final double APPROACH_POWER = 0.45; // Still fast when close

    final double ANGLE_TOLERANCE = 1.8; // TIGHT - very accurate
    final int PIXEL_DEADZONE = 15;      // SMALL - precise tracking

    // Manual control speed
    final double MANUAL_POWER = 0.35;   // Fast manual control

    // ═══════════════════════════════════════════════════════════════════
    // CAMERA CALIBRATION - HuskyLens Specific
    // ═══════════════════════════════════════════════════════════════════

    // Shooter offset: 3.5" up, 4" left of camera
    final int CAMERA_OFFSET_X = 75;  // Negative = aim left (pixels)

    // HuskyLens calibration: 320px wide, ~47° horizontal FOV
    // At typical 4 foot distance: ~0.20 degrees per pixel
    final double PIXELS_TO_DEGREES = 0.20;
    // CALIBRATION: Point at tag, rotate 10°, count pixels moved, adjust this value

    // ═══════════════════════════════════════════════════════════════════
    // SAFETY LIMITS - DISABLED (unlimited rotation)
    // ═══════════════════════════════════════════════════════════════════

    final double MAX_ANGLE_RIGHT = 999999;   // Effectively unlimited
    final double MAX_ANGLE_LEFT = -999999;   // Effectively unlimited
    // WARNING: Make sure cables/wires won't wrap around mechanism!

    // ═══════════════════════════════════════════════════════════════════
    // STATE VARIABLES
    // ═══════════════════════════════════════════════════════════════════

    // PID state
    double integral = 0;
    double lastError = 0;
    double lastMotorPower = 0;
    ElapsedTime pidTimer = new ElapsedTime();

    // Tracking state
    double targetAngle = 0;
    boolean autoTrackEnabled = true;
    boolean aButtonPressed = false;
    int consecutiveLocked = 0;

    // Manual control
    final double MANUAL_SPEED = 2.5; // Degrees per loop

    @Override
    public void runOpMode() {
        // ═══════════════════════════════════════════════════════════════
        // HARDWARE INITIALIZATION
        // ═══════════════════════════════════════════════════════════════

        husky = hardwareMap.get(HuskyLens.class, "h");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretPitch");

        // Configure motor for manual PID control
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // CHANGED - use encoder feedback
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Test motor direction - uncomment if motor runs backwards
        // turretMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  HuskyLens Turret Tracker    ║");
        telemetry.addLine("║  GoBILDA 312 RPM Motor        ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("Initializing HuskyLens...");
        telemetry.update();

        // Initialize HuskyLens with robust error handling
        boolean huskyConnected = false;
        try {
            if (!husky.knock()) {
                telemetry.addLine("❌ HuskyLens not responding!");
                telemetry.addLine("   Check I2C connection");
                telemetry.addLine("   Check power");
                telemetry.update();
                sleep(2000);
            } else {
                huskyConnected = true;
                telemetry.addLine("✓ HuskyLens connected");
                telemetry.update();
                sleep(300);
            }

            sleep(500); // Allow HuskyLens to stabilize
            husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            sleep(300); // Wait for algorithm switch

            telemetry.addLine("✓ TAG_RECOGNITION mode set");
            telemetry.update();
            sleep(300);

        } catch (Exception e) {
            telemetry.addLine("❌ Init error: " + e.getMessage());
            telemetry.update();
            sleep(2000);
        }

        // Display configuration
        telemetry.clear();
        telemetry.addLine("═══ MOTOR SETUP ═══");
        telemetry.addData("Motor", "GoBILDA 312 RPM");
        telemetry.addData("Ticks/Rev", String.format("%.1f", MOTOR_TICKS_PER_REV));
        telemetry.addData("Resolution", String.format("%.2f ticks/°", TICKS_PER_DEGREE));
        telemetry.addData("Gear Ratio", String.format("%.1f:1", GEAR_RATIO));
        telemetry.addLine();
        telemetry.addLine("═══ MOTOR TEST MODE ═══");
        telemetry.addLine("Hold D-Pad RIGHT for 3 seconds");
        telemetry.addLine("Motor should spin and encoder count up");
        telemetry.addLine("If motor doesn't move:");
        telemetry.addLine("  • Check motor is plugged in");
        telemetry.addLine("  • Check config name matches");
        telemetry.addLine("  • Try different motor port");
        telemetry.addLine();
        telemetry.addLine("═══ CAMERA SETUP ═══");
        telemetry.addData("HuskyLens", huskyConnected ? "✓ Connected" : "✗ Error");
        telemetry.addData("Offset", Math.abs(CAMERA_OFFSET_X) + "px LEFT");
        telemetry.addData("Calibration", String.format("%.2f °/px", PIXELS_TO_DEGREES));
        telemetry.addLine();
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("D-Pad R/L - Direct motor test (0.5 power)");
        telemetry.addLine("A - Toggle auto-tracking");
        telemetry.addLine("B - Emergency stop");
        telemetry.addLine("X - Return to center (0°)");
        telemetry.addLine("Y - Reset encoder");
        telemetry.addLine("D-Pad Up - Enable tracking");
        telemetry.addLine();
        telemetry.addLine("✓ Ready! Press START");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        // ═══════════════════════════════════════════════════════════════
        // MAIN CONTROL LOOP
        // ═══════════════════════════════════════════════════════════════

        int loopCount = 0;
        int consecutiveNoBlocks = 0;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            loopCount++;
            loopTimer.reset();

            double currentAngle = getMotorAngleDegrees();

            // ═══════════════════════════════════════════════════════════
            // GAMEPAD CONTROLS
            // ═══════════════════════════════════════════════════════════

            // Emergency stop
            if (gamepad1.b) {
                turretMotor.setPower(0);
                autoTrackEnabled = false;
                resetPID();
                telemetry.clear();
                telemetry.addLine("╔═══════════════════════════════╗");
                telemetry.addLine("║    🛑 EMERGENCY STOP 🛑      ║");
                telemetry.addLine("╚═══════════════════════════════╝");
                telemetry.addData("Current Angle", String.format("%.1f°", currentAngle));
                telemetry.addLine();
                telemetry.addLine("Press A to resume");
                telemetry.update();
                sleep(300);
                continue;
            }

            // Reset encoder to 0
            if (gamepad1.y) {
                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetAngle = 0;
                resetPID();
                telemetry.clear();
                telemetry.addLine("✓ Encoder reset to 0°");
                telemetry.addLine("✓ Target set to 0°");
                telemetry.update();
                sleep(400);
            }

            // Toggle auto-tracking
            if (gamepad1.a && !aButtonPressed) {
                autoTrackEnabled = !autoTrackEnabled;
                aButtonPressed = true;
                resetPID();
                consecutiveLocked = 0;

                if (autoTrackEnabled) {
                    targetAngle = currentAngle; // Start from current position
                } else {
                    turretMotor.setPower(0);
                }
            } else if (!gamepad1.a) {
                aButtonPressed = false;
            }

            // Manual control - SMOOTH CONTINUOUS ROTATION
            boolean manualControl = false;
            if (gamepad1.dpad_left) {
                turretMotor.setPower(-MANUAL_POWER); // Smooth left rotation
                manualControl = true;
                autoTrackEnabled = false;
                resetPID();
                consecutiveLocked = 0;
            } else if (gamepad1.dpad_right) {
                turretMotor.setPower(MANUAL_POWER); // Smooth right rotation
                manualControl = true;
                autoTrackEnabled = false;
                resetPID();
                consecutiveLocked = 0;
            } else if (!autoTrackEnabled && !gamepad1.x) {
                // Only stop if not in auto mode and not pressing X
                if (!manualControl) {
                    turretMotor.setPower(0);
                }
            }
            if (gamepad1.dpad_up) {
                autoTrackEnabled = true;
                targetAngle = currentAngle;
                resetPID();
                consecutiveLocked = 0;
            }

            // Return to center
            if (gamepad1.x) {
                targetAngle = 0;
                autoTrackEnabled = false;
                resetPID();
                consecutiveLocked = 0;
            }

            // Enforce safety limits
            targetAngle = Range.clip(targetAngle, MAX_ANGLE_LEFT, MAX_ANGLE_RIGHT);

            // ═══════════════════════════════════════════════════════════
            // HUSKYLENS READING
            // ═══════════════════════════════════════════════════════════

            HuskyLens.Block[] blocks = null;
            int numBlocks = 0;

            try {
                blocks = husky.blocks();
                numBlocks = (blocks != null) ? blocks.length : 0;
                consecutiveNoBlocks = (numBlocks == 0) ? consecutiveNoBlocks + 1 : 0;
            } catch (Exception e) {
                consecutiveNoBlocks++;
                numBlocks = 0;
            }

            // ═══════════════════════════════════════════════════════════
            // AUTO-TRACKING LOGIC
            // ═══════════════════════════════════════════════════════════

            telemetry.clear();

            if (autoTrackEnabled && !manualControl) {
                if (numBlocks > 0) {
                    // Get first detected tag
                    HuskyLens.Block tag = blocks[0];

                    int tagX = tag.x;
                    int centerX = 160; // HuskyLens center (320px / 2)
                    int targetX = centerX + CAMERA_OFFSET_X;
                    int pixelError = tagX - targetX;

                    if (Math.abs(pixelError) > PIXEL_DEADZONE) {
                        // Outside deadzone - active tracking
                        double angleError = pixelError * PIXELS_TO_DEGREES;

                        // Update target angle
                        targetAngle = currentAngle + angleError;
                        targetAngle = Range.clip(targetAngle, MAX_ANGLE_LEFT, MAX_ANGLE_RIGHT);

                        consecutiveLocked = 0;

                        // Display tracking status
                        telemetry.addLine("╔═══ 🎯 TRACKING ACTIVE ═══╗");
                        telemetry.addLine("║");
                        telemetry.addData("║ AprilTag ID", tag.id);
                        telemetry.addData("║ Tag X Position", tagX + "px");
                        telemetry.addData("║ Target X Position", targetX + "px");
                        telemetry.addData("║ Pixel Error", pixelError + "px");
                        telemetry.addData("║ → Angle Error", String.format("%.2f°", angleError));
                        telemetry.addLine("║");

                        // Status based on error magnitude
                        if (Math.abs(pixelError) > 80) {
                            telemetry.addLine("║ Status: ⚡ FAR - Moving fast");
                        } else if (Math.abs(pixelError) > 40) {
                            telemetry.addLine("║ Status: 🔍 CLOSE - Slowing down");
                        } else {
                            telemetry.addLine("║ Status: 🎯 CONVERGING");
                        }

                    } else {
                        // Within deadzone - confirming lock
                        consecutiveLocked++;

                        if (consecutiveLocked >= 3) {
                            // Fully locked on - FORCE STOP for precision
                            turretMotor.setPower(0);
                            resetPID();

                            // Display locked status
                            telemetry.addLine("╔═══ ✅ LOCKED ON TARGET ═══╗");
                            telemetry.addLine("║");
                            telemetry.addData("║ ✓ AprilTag ID", tag.id);
                            telemetry.addData("║ ✓ Pixel Error", pixelError + "px");
                            telemetry.addData("║ ✓ Angle Error", String.format("%.2f°", targetAngle - currentAngle));
                            telemetry.addLine("║");
                            telemetry.addData("║ ✓ Offset Applied", Math.abs(CAMERA_OFFSET_X) + "px LEFT");
                            telemetry.addLine("║");
                            telemetry.addLine("║ 🎯 SHOOTER ALIGNED");
                            telemetry.addLine("║ ✓✓ READY TO FIRE ✓✓");
                        } else {
                            // Stabilizing lock
                            telemetry.addLine("╔═══ 🎯 LOCKING ON ═══╗");
                            telemetry.addLine("║");
                            telemetry.addData("║ AprilTag ID", tag.id);
                            telemetry.addData("║ Pixel Error", pixelError + "px");
                            telemetry.addData("║ Stability", consecutiveLocked + "/3 loops");
                            telemetry.addLine("║ Confirming lock...");
                        }
                    }

                } else {
                    // No tags detected
                    consecutiveLocked = 0;

                    telemetry.addLine("╔═══ ❌ NO TARGET DETECTED ═══╗");
                    telemetry.addLine("║");
                    telemetry.addLine("║ No AprilTag in view");
                    telemetry.addLine("║ Point HuskyLens at tag");

                    if (consecutiveNoBlocks > 50) {
                        telemetry.addLine("║");
                        telemetry.addLine("║ ⚠️ Troubleshooting:");
                        telemetry.addLine("║   • Tag in camera view?");
                        telemetry.addLine("║   • Lighting adequate?");
                        telemetry.addLine("║   • HuskyLens powered?");
                        telemetry.addLine("║   • Algorithm set correctly?");
                    }
                }

            } else if (manualControl) {
                telemetry.addLine("╔═══ 🎮 MANUAL CONTROL ═══╗");
                telemetry.addLine("║");
                telemetry.addLine("║ Using D-Pad controls");
                telemetry.addData("║ Power", gamepad1.dpad_left ? "-" + MANUAL_POWER : "+" + MANUAL_POWER);
                telemetry.addLine("║ Press D-Pad Up for auto");

            } else {
                telemetry.addLine("╔═══ ⏸ STANDBY MODE ═══╗");
                telemetry.addLine("║");
                telemetry.addLine("║ Auto-tracking disabled");
                telemetry.addLine("║ Press A to enable");
            }

            // ═══════════════════════════════════════════════════════════
            // PID MOTOR CONTROL
            // ═══════════════════════════════════════════════════════════

            double positionError = targetAngle - currentAngle;
            double motorPower = calculatePID(positionError);

            // Smooth power ramping (prevents jerks)
            double smoothedPower = lastMotorPower + (motorPower - lastMotorPower) * 0.4;
            lastMotorPower = smoothedPower;

            turretMotor.setPower(smoothedPower);

            // ═══════════════════════════════════════════════════════════
            // STATUS TELEMETRY
            // ═══════════════════════════════════════════════════════════

            telemetry.addLine("║");
            telemetry.addLine("╚═══════════════════════════════╝");
            telemetry.addLine();
            telemetry.addLine("═══ MOTOR STATUS ═══");
            telemetry.addData("Mode", autoTrackEnabled ? "AUTO" : "MANUAL");
            telemetry.addData("Current Angle", String.format("%.1f°", currentAngle));
            telemetry.addData("Target Angle", String.format("%.1f°", targetAngle));
            telemetry.addData("Position Error", String.format("%.1f°", positionError));
            telemetry.addData("Motor Power", String.format("%.3f", motorPower));
            telemetry.addData("Power Command", motorPower >= 0 ? "FORWARD" : "REVERSE");
            telemetry.addData("Encoder Ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Encoder Changing?", turretMotor.getCurrentPosition() != 0 ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addLine("═══ SYSTEM STATUS ═══");
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Loop Time", String.format("%.1f ms", loopTimer.milliseconds()));
            telemetry.addData("HuskyLens Blocks", numBlocks);

            // Safety warnings - now only for information
            if (Math.abs(currentAngle) > 360) {
                telemetry.addLine();
                telemetry.addLine("ℹ️ INFO: Multiple rotations (" + String.format("%.0f", currentAngle/360) + " turns)");
            }

            if (Math.abs(currentAngle) > 720) {
                telemetry.addLine("⚠️ WARNING: Check for cable wrapping!");
            }

            if (consecutiveNoBlocks > 100) {
                telemetry.addLine();
                telemetry.addLine("⚠️ HuskyLens connection issue?");
            }

            telemetry.update();

            // Target 50Hz loop rate for maximum responsiveness
            long loopTime = (long) loopTimer.milliseconds();
            if (loopTime < 20) {
                sleep(20 - loopTime);
            }
        }

        // Cleanup
        turretMotor.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════
    // HELPER FUNCTIONS
    // ═══════════════════════════════════════════════════════════════════

    private double getMotorAngleDegrees() {
        int ticks = turretMotor.getCurrentPosition();
        return ticks / TICKS_PER_DEGREE;
    }

    private double calculatePID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Clamp dt to reasonable values
        if (dt < 0.008) dt = 0.020;
        if (dt > 0.12) dt = 0.020;

        // Proportional term - AGGRESSIVE for speed
        double pTerm = error * KP;

        // Integral term - accumulates faster for quick settling
        if (Math.abs(error) < ANGLE_TOLERANCE * 5) {
            integral += error * dt;
            integral = Range.clip(integral, -50, 50);

            // Partial reset on direction change - keeps momentum
            if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
                integral *= 0.15; // Keep some history for smooth reversals
            }
        } else {
            integral *= 0.85; // Slow decay when far - maintains velocity
        }
        double iTerm = integral * KI;

        // Derivative term - STRONG damping prevents overshoot at high speeds
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        derivative = Range.clip(derivative, -150, 150);
        double dTerm = derivative * KD;

        lastError = error;

        // Calculate total power
        double power = pTerm + iTerm + dTerm;

        // Dynamic power limiting - stay fast even when close
        double maxPowerNow = (Math.abs(error) < 5) ? APPROACH_POWER : MAX_POWER;

        // Apply minimum threshold - ensures instant response
        if (Math.abs(error) > ANGLE_TOLERANCE) {
            if (Math.abs(power) > 0 && Math.abs(power) < MIN_POWER) {
                power = Math.signum(power) * MIN_POWER;
            }
        } else {
            // Within tolerance - full stop for accuracy
            power = 0;
            integral *= 0.5; // Reduce integral when stopped
        }

        // Clamp to limits
        return Range.clip(power, -maxPowerNow, maxPowerNow);
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastMotorPower = 0;
        pidTimer.reset();
    }
}