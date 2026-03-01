package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.List;

@Autonomous(name="UseThisAuto", group="Competition")
public class NanoAuto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor roller, turretPitch;
    private Servo kicker, turretYaw;
    private ServoImplEx sortWheel;
    private ColorSensor cs;
    private Limelight3A limelight;

    private double calculatedShooterPower = 0.65;
    private double lastValidDistance = 72.0;
    private double yawPosition = 0.5;

    // ✅ FIXED: Lock storage with better tracking
    private double lockedYawPosition = 0.5;
    private double lockedPitchPosition = 0; // Changed from power to position
    private boolean isLocked = false;
    private ElapsedTime lockAgeTimer = new ElapsedTime();
    private static final double LOCK_EXPIRY_MS = 3000; // Lock expires after 3s

    // ✅ FIXED: Motor-specific constants (CONFIGURE THESE FOR YOUR MOTOR!)
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // REV HD Hex Motor = 28, adjust for your motor
    private static final double SHOOTER_GEAR_RATIO = 1.0; // Set to your gear ratio (e.g., 2.0 if 2:1)

    private static final double BALL_1_POWER_MULTIPLIER = 0.95;
    private static final double BALL_2_POWER_MULTIPLIER = 1.0;
    private static final double BALL_3_POWER_MULTIPLIER = 0.95;

    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 80;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.35;

    // ✅ FIXED: Shooter speed constants with proper RPM calculation
    private static final double TARGET_SHOOTER_RPM = 1500; // Base target RPM
    private static final double SHOOTER_RPM_TOLERANCE = 100; // ±100 RPM tolerance
    private static final int SHOOTER_SPEED_CHECK_TIMEOUT = 1500; // Reduced from 2000ms
    private static final int MIN_STABLE_SPEED_TIME = 100; // Must be stable for 100ms

    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;
    private boolean ballDetectedInSlot = false;
    private boolean inCooldown = false;
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();

    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.38;
    private static final double SLOT_2_POSITION = 0.78;
    private static final double SHOOT_POSITION_0 = 0.57;
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;

    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    // ✅ ANTI-WAVERING: Reduced gains to prevent oscillation
    private static final double YAW_SERVO_GAIN = -0.008; // Further reduced for stability

    // ✅ FIXED: Turret pitch now uses target position instead of continuous power
    private static final int PITCH_TICKS_PER_DEGREE = 20; // CALIBRATE THIS!
    private static final double PITCH_POWER = 0.6;

    private static final double TX_GAIN = 0.018; // Further reduced to prevent overshoot
    private static final double TY_GAIN = 0.8; // Now used for position calculation

    // ✅ ANTI-WAVERING: Deadband and smoothing improvements
    private static final double AUTO_AIM_TIMEOUT = 2500;
    private static final double TARGET_TOLERANCE_TX = 1.5; // Wider tolerance reduces chasing
    private static final double TARGET_TOLERANCE_TY = 1.5;
    private static final double DEADBAND_TX = 0.3; // Ignore tiny errors to prevent hunting
    private static final double DEADBAND_TY = 0.3;
    private static final int STABLE_FRAMES_REQUIRED = 8; // 80ms for more stable lock
    private static final double SMOOTHING_FACTOR = 0.85; // Stronger smoothing: 85% old + 15% new
    private static final double MAX_SERVO_CHANGE_PER_FRAME = 0.01; // Rate limit servo movement

    private static final int SERVO_MOVE_DELAY = 750;
    private static final int SHOOT_DELAY = 375;
    private static final int KICKER_CLOSE_DELAY = 300;
    private static final int BETWEEN_SHOTS_DELAY = 200;
    private static final int MOTOR_SPINUP_TIME = 1000;

    // ✅ IMPROVED: Drive calibration system
    private static final double INCHES_PER_SECOND_BASE = 26; // Your current value
    private static final double DEGREES_PER_SECOND_BASE = 95; // Your current value

    // Calibration multipliers (start at 1.0, adjust based on testing)
    private double driveCalibration = 1.0;
    private double turnCalibration = 1.0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime test = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor Config", "%.0f ticks/rev, %.1fx gear",
                SHOOTER_TICKS_PER_REV, SHOOTER_GEAR_RATIO);
        telemetry.update();

        waitForStart();
        runtime.reset();
        test.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Detecting balls & scanning motif");
            telemetry.update();

            autoDetectBalls(3000);

            telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);
            if (motifDetected) {
                telemetry.addData("Motif", "Tag %d - %s", detectedAprilTagId, getPatternString());
            } else {
                telemetry.addData("Motif", "Not detected");
            }
            telemetry.update();
            driveForward(6,0.4);
            sleep(50);
            turnRight(15, 0.4);
            sleep(50);

            shooter.setPower(0.65);
            sleep(500);

            telemetry.addData("Status", "Aiming...");
            telemetry.update();

            boolean locked = autoAimAndLock();

            telemetry.addData("Locked", locked ? "Yes" : "No");
            telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
            telemetry.addData("Distance", "%.1f in", lastValidDistance);
            telemetry.addData("Yaw Position", "%.3f", yawPosition);
            telemetry.update();

            if (!locked) {
                shooter.setPower(0.7);
                calculatedShooterPower = 0.7;
            }

            shooter.setPower(calculatedShooterPower);
            sleep(1000);

            test.reset();
            if (motifDetected) {
                shootBallsWithPattern(targetPattern);
            } else {
                shootBallsOptimized();
            }

            stopShooter();
            turnLeft(15,0.4);
            driveForward(12,0.4);


            telemetry.addData("Complete", "%.1fs", test.seconds());
            telemetry.update();
            sleep(2000);
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        sortWheel = hardwareMap.get(ServoImplEx.class, "sortWheel");
        sortWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_CLOSED);
        sortWheel.setPosition(SLOT_0_POSITION);

        cs = hardwareMap.get(ColorSensor.class, "cs");

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretYaw.setPosition(YAW_CENTER);

        // ✅ FIXED: Turret pitch now uses RUN_TO_POSITION mode
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setTargetPosition(0);
        turretPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretPitch.setPower(PITCH_POWER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        ballCount = 0;
        currentSlot = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
    }

    // ✅ FIXED: Proper RPM calculation
    private double getShooterRPM() {
        double ticksPerSecond = Math.abs(shooter.getVelocity());
        double revsPerSecond = ticksPerSecond / SHOOTER_TICKS_PER_REV;
        double motorRPM = revsPerSecond * 60.0;
        return motorRPM / SHOOTER_GEAR_RATIO; // Account for gear ratio
    }

    // ✅ FIXED: Calculate target RPM based on power
    private double getTargetRPM(double power) {
        return TARGET_SHOOTER_RPM * power;
    }

    // ============================================
    // ✅ FIXED: IMPROVED AUTO-AIM WITH POSITION-BASED PITCH
    // ============================================
    public boolean autoAimAndLock() {
        ElapsedTime lockTimer = new ElapsedTime();
        lockTimer.reset();

        double smoothedTx = 0, smoothedTy = 0;
        boolean firstReading = true;
        int stableCount = 0;
        double lastYawPosition = YAW_CENTER; // Track for rate limiting

        // Reset turret to center
        yawPosition = YAW_CENTER;
        turretYaw.setPosition(YAW_CENTER);
        turretPitch.setTargetPosition(0);
        sleep(150);

        while (opModeIsActive() && lockTimer.milliseconds() < AUTO_AIM_TIMEOUT) {
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                sleep(10);
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                sleep(10);
                continue;
            }

            // Find goal tag (ID 20 or 24)
            LLResultTypes.FiducialResult target = null;
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = (int) f.getFiducialId();
                if (id == 20 || id == 24) {
                    target = f;
                    break;
                }
            }

            if (target == null) {
                sleep(10);
                continue;
            }

            double tx = target.getTargetXDegrees();
            double ty = target.getTargetYDegrees();
            double ta = target.getTargetArea();

            // Initialize smoothing on first valid reading
            if (firstReading) {
                smoothedTx = tx;
                smoothedTy = ty;
                firstReading = false;
            }

            // ✅ ANTI-WAVERING: Much stronger smoothing (85/15)
            smoothedTx = smoothedTx * SMOOTHING_FACTOR + tx * (1 - SMOOTHING_FACTOR);
            smoothedTy = smoothedTy * SMOOTHING_FACTOR + ty * (1 - SMOOTHING_FACTOR);

            // Update shooter power
            double distance = calculateDistance(ta);
            calculatedShooterPower = calculateShooterPower(distance);
            lastValidDistance = distance;

            // ✅ ANTI-WAVERING: Apply deadband to prevent micro-corrections
            double effectiveTx = (Math.abs(smoothedTx) < DEADBAND_TX) ? 0 : smoothedTx;
            double effectiveTy = (Math.abs(smoothedTy) < DEADBAND_TY) ? 0 : smoothedTy;

            // ✅ ANTI-WAVERING: YAW servo with rate limiting
            double targetYawPosition =YAW_CENTER - (effectiveTx * YAW_SERVO_GAIN); // ✅ Subtract instead of add

            targetYawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, targetYawPosition));

            // Rate limit: max change per frame
            double yawChange = targetYawPosition - lastYawPosition;
            if (Math.abs(yawChange) > MAX_SERVO_CHANGE_PER_FRAME) {
                yawChange = Math.signum(yawChange) * MAX_SERVO_CHANGE_PER_FRAME;
            }
            yawPosition = lastYawPosition + yawChange;
            lastYawPosition = yawPosition;

            turretYaw.setPosition(yawPosition);

            // ✅ ANTI-WAVERING: PITCH motor with deadband
            int pitchTargetTicks = (int)(-effectiveTy * TY_GAIN * PITCH_TICKS_PER_DEGREE);
            turretPitch.setTargetPosition(pitchTargetTicks);

            // ✅ FIXED: Check stability with proper frame counting
            boolean onTarget = (Math.abs(smoothedTx) < TARGET_TOLERANCE_TX) &&
                    (Math.abs(smoothedTy) < TARGET_TOLERANCE_TY);

            if (onTarget) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            telemetry.addData("TX", "%.2f°", smoothedTx);
            telemetry.addData("TY", "%.2f°", smoothedTy);
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Power", "%.3f", calculatedShooterPower);
            telemetry.addData("Stable", "%d/%d", stableCount, STABLE_FRAMES_REQUIRED);
            telemetry.addData("Pitch Pos", "%d ticks", pitchTargetTicks);
            telemetry.addData("Deadband Active", (effectiveTx == 0 && effectiveTy == 0) ? "YES" : "NO");
            telemetry.update();

            // ✅ FIXED: Lock on sustained stability (80ms now)
            if (stableCount >= STABLE_FRAMES_REQUIRED) {
                lockedYawPosition = yawPosition;
                lockedPitchPosition = pitchTargetTicks;
                isLocked = true;
                lockAgeTimer.reset();

                telemetry.addData("Status", "🔒 LOCKED!");
                telemetry.addData("Time", "%.0f ms", lockTimer.milliseconds());
                telemetry.addData("Locked Yaw", "%.4f", lockedYawPosition);
                telemetry.addData("Locked Pitch", "%d ticks", (int)lockedPitchPosition);
                telemetry.update();
                return true;
            }

            sleep(10); // 10ms loop for ~100Hz update rate
        }

        isLocked = false;
        telemetry.addData("Status", "⏱ TIMEOUT");
        telemetry.update();
        return false;
    }

    // ✅ FIXED: Apply locked aim with expiry check
    public void applyLockedAim() {
        if (!isLocked) {
            telemetry.addData("WARNING", "No lock stored!");
            telemetry.update();
            return;
        }

        // Check if lock has expired
        if (lockAgeTimer.milliseconds() > LOCK_EXPIRY_MS) {
            telemetry.addData("WARNING", "Lock expired! Re-aiming...");
            telemetry.update();
            isLocked = false;
            return;
        }

        turretYaw.setPosition(lockedYawPosition);
        turretPitch.setTargetPosition((int)lockedPitchPosition);

        telemetry.addData("Applied", "Locked aim restored");
        telemetry.addData("Yaw", "%.4f", lockedYawPosition);
        telemetry.addData("Pitch", "%d ticks", (int)lockedPitchPosition);
        telemetry.addData("Lock Age", "%.1fs", lockAgeTimer.seconds());
        telemetry.update();
    }

    public void resetLock() {
        isLocked = false;
        lockedYawPosition = YAW_CENTER;
        lockedPitchPosition = 0;
        turretPitch.setTargetPosition(0);
        telemetry.addData("Status", "Lock reset");
        telemetry.update();
    }

    public boolean isTargetLocked() {
        return isLocked && (lockAgeTimer.milliseconds() <= LOCK_EXPIRY_MS);
    }

    public void shootBallsOptimized() {
        shootBallsWithPattern(null);
    }

    public void shootBallsWithPattern(String[] targetPattern) {
        int[] shootOrder = (targetPattern == null) ? new int[]{0, 1, 2} : calculateShootOrder(targetPattern);

        telemetry.addData("Shoot Order", "[%d, %d, %d]", shootOrder[0], shootOrder[1], shootOrder[2]);
        telemetry.update();
        sleep(500);

        // Ball 1
        telemetry.addData("Shooting", "Ball 1 from slot %d", shootOrder[0]);
        telemetry.update();
        int slot1 = shootOrder[0];
        double ball1Power = calculatedShooterPower * BALL_1_POWER_MULTIPLIER;
        shooter.setPower(ball1Power);
        sortWheel.setPosition(getShootPositionForSlot(slot1));
        sleep(SERVO_MOVE_DELAY * 2);
        kicker.setPosition(KICKER_OPEN);
        sleep(SHOOT_DELAY);
        kicker.setPosition(KICKER_CLOSED);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_CLOSED);
        sleep(BETWEEN_SHOTS_DELAY);

        // Ball 2
        telemetry.addData("Shooting", "Ball 2 from slot %d", shootOrder[1]);
        telemetry.update();
        int slot2 = shootOrder[1];
        double ball2Power = calculatedShooterPower * BALL_2_POWER_MULTIPLIER;
        shooter.setPower(ball2Power);
        sortWheel.setPosition(getShootPositionForSlot(slot2));
        sleep(SERVO_MOVE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(SHOOT_DELAY);
        kicker.setPosition(KICKER_CLOSED);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_CLOSED);
        sleep(BETWEEN_SHOTS_DELAY);

        // Ball 3
        telemetry.addData("Shooting", "Ball 3 from slot %d", shootOrder[2]);
        telemetry.update();
        telemetry.update();
        int slot3 = shootOrder[2];
        double ball3Power = calculatedShooterPower * BALL_3_POWER_MULTIPLIER;
        shooter.setPower(ball3Power);
        sortWheel.setPosition(getShootPositionForSlot(slot3));
        sleep(SERVO_MOVE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(SHOOT_DELAY + 100);
        kicker.setPosition(KICKER_CLOSED);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_OPEN);  // ✅ Added second tap
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_CLOSED);

        telemetry.addData("Complete", "All 3 balls fired");
        telemetry.update();
    }

    // ============================================
    // ✅ IMPROVED: DRIVE CALIBRATION SYSTEM
    // ============================================

    /**
     * Call this during testing to adjust drive calibration
     * Example: if robot drives 40" when asked for 36", call setCalibratedDistance(36, 40)
     */
    public void setCalibratedDistance(double targetInches, double actualInches) {
        driveCalibration = targetInches / actualInches;
        telemetry.addData("Drive Cal", "%.3f (%.1f→%.1f)",
                driveCalibration, targetInches, actualInches);
        telemetry.update();
    }

    /**
     * Call this during testing to adjust turn calibration
     * Example: if robot turns 50° when asked for 45°, call setCalibratedTurn(45, 50)
     */
    public void setCalibratedTurn(double targetDegrees, double actualDegrees) {
        turnCalibration = targetDegrees / actualDegrees;
        telemetry.addData("Turn Cal", "%.3f (%.1f→%.1f)",
                turnCalibration, targetDegrees, actualDegrees);
        telemetry.update();
    }

    public void driveForward(double inches, double power) {
        double calibratedInches = inches * driveCalibration;
        int ms = (int)((calibratedInches / INCHES_PER_SECOND_BASE) * 1000 / power);
        mecanum(power, 0, 0);
        sleep(ms);
        stopDrive();
    }

    public void driveBackward(double inches, double power) {
        double calibratedInches = inches * driveCalibration;
        int ms = (int)((calibratedInches / INCHES_PER_SECOND_BASE) * 1000 / power);
        mecanum(-power, 0, 0);
        sleep(ms);
        stopDrive();
    }

    public void strafeRight(double inches, double power) {
        double calibratedInches = inches * driveCalibration;
        int ms = (int)((calibratedInches / INCHES_PER_SECOND_BASE) * 1000 / power);
        mecanum(0, power, 0);
        sleep(ms);
        stopDrive();
    }

    public void strafeLeft(double inches, double power) {
        double calibratedInches = inches * driveCalibration;
        int ms = (int)((calibratedInches / INCHES_PER_SECOND_BASE) * 1000 / power);
        mecanum(0, -power, 0);
        sleep(ms);
        stopDrive();
    }

    public void turnRight(double degrees, double power) {
        double calibratedDegrees = degrees * turnCalibration;
        int ms = (int)((calibratedDegrees / DEGREES_PER_SECOND_BASE) * 1000 / power);
        mecanum(0, 0, power);
        sleep(ms);
        stopDrive();
    }

    public void turnLeft(double degrees, double power) {
        double calibratedDegrees = degrees * turnCalibration;
        int ms = (int)((calibratedDegrees / DEGREES_PER_SECOND_BASE) * 1000 / power);
        mecanum(0, 0, -power);
        sleep(ms);
        stopDrive();
    }

    // ============================================
    // REMAINING METHODS (unchanged)
    // ============================================

    public void autoDetectBalls(int timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        currentSlot = 0;
        sortWheel.setPosition(SLOT_0_POSITION);
        ballDetectedInSlot = false;
        inCooldown = false;
        roller.setPower(INTAKE_POWER);

        while (opModeIsActive() && timer.milliseconds() < timeoutMs && ballCount < MAX_BALLS) {
            autoDetectAndRotate();
            if (!motifDetected) {
                scanAprilTagPattern();
            }
            telemetry.addData("Detecting", "%d/%d balls", ballCount, MAX_BALLS);
            if (motifDetected) {
                telemetry.addData("Motif Found", "Tag %d", detectedAprilTagId);
            }
            telemetry.update();
            sleep(25);
        }
        roller.setPower(0);
    }

    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS) return;

        if (inCooldown) {
            if (cooldownTimer.milliseconds() > DETECTION_COOLDOWN) {
                inCooldown = false;
            } else {
                return;
            }
        }

        int red = cs.red();
        int green = cs.green();
        int blue = cs.blue();

        boolean ballPresent = (red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD);

        if (ballPresent && !ballDetectedInSlot && !inCooldown) {
            String detectedColor = detectBallColor(red, green, blue);
            if (detectedColor != null) {
                ballSlots[currentSlot] = detectedColor;
                ballCount++;
                ballDetectedInSlot = true;
                detectionTimer.reset();
            }
        }

        if (ballDetectedInSlot && detectionTimer.milliseconds() > AUTO_ROTATE_DELAY) {
            if (ballCount < MAX_BALLS) {
                rotateSortWheelToNextSlot();
                inCooldown = true;
                cooldownTimer.reset();
            }
            ballDetectedInSlot = false;
        }
    }

    private String detectBallColor(int red, int green, int blue) {
        if (green > red && green > blue && green > COLOR_THRESHOLD) {
            return "Green";
        } else if ((red > green && blue > green && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD)) ||
                (red + blue > green * 2 && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))) {
            return "Purple";
        }
        return null;
    }

    private void rotateSortWheelToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;
        switch (currentSlot) {
            case 0: sortWheel.setPosition(SLOT_0_POSITION); break;
            case 1: sortWheel.setPosition(SLOT_1_POSITION); break;
            case 2: sortWheel.setPosition(SLOT_2_POSITION); break;
        }
    }

    private void scanAprilTagPattern() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                break;
            }
        }
    }

    private void setTargetPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21:
                targetPattern = new String[]{"Green", "Purple", "Purple"};
                break;
            case 22:
                targetPattern = new String[]{"Purple", "Green", "Purple"};
                break;
            case 23:
                targetPattern = new String[]{"Purple", "Purple", "Green"};
                break;
            default:
                targetPattern = null;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s, %s, %s]", targetPattern[0], targetPattern[1], targetPattern[2]);
    }

    public void setBallPattern(String slot0, String slot1, String slot2) {
        ballSlots[0] = slot0;
        ballSlots[1] = slot1;
        ballSlots[2] = slot2;
        ballCount = 3;
    }

    private int[] calculateShootOrder(String[] targetPattern) {
        int[] shootOrder = new int[3];
        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) availableBalls[i] = ballSlots[i];

        for (int patternIndex = 0; patternIndex < 3; patternIndex++) {
            String requiredColor = targetPattern[patternIndex];
            int slotWithRequiredBall = -1;

            for (int slot = 0; slot < 3; slot++) {
                if (availableBalls[slot] != null && availableBalls[slot].equals(requiredColor)) {
                    slotWithRequiredBall = slot;
                    availableBalls[slot] = null;
                    break;
                }
            }

            if (slotWithRequiredBall == -1) {
                shootOrder[patternIndex] = 0;
            } else {
                shootOrder[patternIndex] = slotWithRequiredBall;
            }
        }
        return shootOrder;
    }

    private double getShootPositionForSlot(int slot) {
        switch (slot) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
        }
    }

    public void aimTurretYaw(double yawAngle) {
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawAngle));
        turretYaw.setPosition(yawPosition);
    }

    public void centerTurret() {
        yawPosition = YAW_CENTER;
        turretYaw.setPosition(YAW_CENTER);
        turretPitch.setTargetPosition(0);
    }

    public void spinUpShooter() {
        shooter.setPower(calculatedShooterPower);
        sleep(MOTOR_SPINUP_TIME);
    }

    public void stopShooter() {
        shooter.setPower(0);
    }

    public void stopDrive() {
        mecanum(0, 0, 0);
    }

    public void customDrive(double drive, double strafe, double turn, int milliseconds) {
        mecanum(drive, strafe, turn);
        sleep(milliseconds);
        stopDrive();
    }

    public double getShooterPower() {
        return calculatedShooterPower;
    }

    public double getDistanceToTarget() {
        return lastValidDistance;
    }

    public void pause(int milliseconds) {
        sleep(milliseconds);
    }

    private void mecanum(double drive, double strafe, double turn) {
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;

        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private double calculateDistance(double targetArea) {
        if (targetArea < 0.001) return lastValidDistance;
        double rawDistance = (0.55 / targetArea) - 12.2047;
        return Math.max(14.0, Math.min(144.0, rawDistance));
    }

    private double calculateShooterPower(double distance) {
        double minPower = 0.57;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 144;
        double power = minPower + (distance - minDist) * (maxPower - minPower) / (maxDist - minDist);
        return Math.max(minPower, Math.min(maxPower, power));
    }
}