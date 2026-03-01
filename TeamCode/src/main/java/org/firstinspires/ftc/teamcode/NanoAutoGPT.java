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

@Autonomous(name="AutoAimAutonomousNew", group="Competition")
public class NanoAutoGPT extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor roller, turretPitch;
    private Servo kicker, turretYaw;
    private ServoImplEx sortWheel;
    private ColorSensor cs;
    private Limelight3A limelight;

    private double calculatedShooterPower = 0.8;
    private double lastValidDistance = 72.0;
    private double yawPosition = 0.5;

    private static final double BALL_1_POWER_MULTIPLIER = 0.95;
    private static final double BALL_2_POWER_MULTIPLIER = 1.0;
    private static final double BALL_3_POWER_MULTIPLIER = 0.95;
    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 80;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.35;

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
    private static final double YAW_SERVO_GAIN = 0.025;  // Faster response

    private static final double TURRET_PITCH_SPEED = 0.6;  // Faster pitch
    private static final double TX_GAIN = 0.04;  // Faster tracking
    private static final double TY_GAIN = 0.04;  // Faster tracking
    private static final double AUTO_AIM_TIMEOUT = 4000;  // Quick lock attempt
    private static final double TARGET_TOLERANCE_TX = 4.0;  // More forgiving
    private static final double TARGET_TOLERANCE_TY = 4.0;

    // MATCHED TO TELEOP TIMING
    private static final int SERVO_MOVE_DELAY = 750;
    private static final int SHOOT_DELAY = 375;
    private static final int KICKER_CLOSE_DELAY = 300;
    private static final int BETWEEN_SHOTS_DELAY = 200;
    private static final int MOTOR_SPINUP_TIME = 1000;

    private static final double INCHES_PER_SECOND = 26;
    private static final double DEGREES_PER_SECOND = 95;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime test = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
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
                telemetry.addData("Motif", "Tag %d detected", detectedAprilTagId);
                telemetry.addData("Pattern", getPatternString());
            } else {
                telemetry.addData("Motif", "None - default order");
            }
            telemetry.update();
            sleep(1000);

            driveForward(36.0, 0.6);
            sleep(50);
            turnRight(45, 0.4);
            sleep(50);

            // Start shooter AFTER positioning
            shooter.setPower(0.65);  // Start with default power
            sleep(500);

            telemetry.addData("Status", "Aiming...");
            telemetry.update();

            boolean locked = autoAimAndLock();

            telemetry.addData("Locked", locked ? "Yes" : "No");
            telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
            telemetry.addData("Distance", "%.1f in", lastValidDistance);
            telemetry.addData("Yaw Position", "%.3f", yawPosition);
            telemetry.update();

            if(locked==false){
                shooter.setPower(0.7);
                calculatedShooterPower = 0.7;
            }

            // Ensure shooter is at correct power
            shooter.setPower(calculatedShooterPower);
            sleep(1000);

            test.reset();
            if (motifDetected) {
                shootBallsWithPattern(targetPattern);
            } else {
                shootBallsOptimized();
            }

            stopShooter();

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

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        ballCount = 0;
        currentSlot = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
    }

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
            case 0:
                sortWheel.setPosition(SLOT_0_POSITION);
                break;
            case 1:
                sortWheel.setPosition(SLOT_1_POSITION);
                break;
            case 2:
                sortWheel.setPosition(SLOT_2_POSITION);
                break;
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

            if (slotWithRequiredBall == -1) return new int[]{0, 1, 2};

            shootOrder[patternIndex] = slotWithRequiredBall;
        }

        return shootOrder;
    }

    public boolean autoAimAndLock() {
        ElapsedTime lockTimer = new ElapsedTime();
        lockTimer.reset();

        double smoothedTx = 0, smoothedTy = 0;
        boolean firstReading = true;
        int stableCount = 0;
        int validReadings = 0;
        int missedReadings = 0;

        yawPosition = YAW_CENTER;
        turretYaw.setPosition(YAW_CENTER);
        sleep(300);

        while (opModeIsActive() && lockTimer.milliseconds() < AUTO_AIM_TIMEOUT) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addData("Limelight", "No valid result");
                telemetry.update();
                missedReadings++;
                if (missedReadings > 20) {
                    telemetry.addData("ERROR", "Lost Limelight connection");
                    telemetry.update();
                    return false;
                }
                sleep(25);  // Faster retry
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addData("Limelight", "No fiducials detected");
                telemetry.update();
                missedReadings++;
                sleep(25);  // Faster retry
                continue;
            }

            LLResultTypes.FiducialResult target = null;
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = (int) f.getFiducialId();
                telemetry.addData("Detected Tag", id);
                if (id == 20 || id == 24) {
                    target = f;
                    break;
                }
            }

            if (target == null) {
                telemetry.addData("Target", "Goal tag not found (need 20 or 24)");
                telemetry.update();
                missedReadings++;
                sleep(25);  // Faster retry
                continue;
            }

            // Reset missed readings counter on successful detection
            missedReadings = 0;
            validReadings++;

            double tx = target.getTargetXDegrees();
            double ty = target.getTargetYDegrees();
            double ta = target.getTargetArea();

            if (firstReading) {
                smoothedTx = tx;
                smoothedTy = ty;
                firstReading = false;
            }

            // Faster smoothing for quicker response
            smoothedTx = smoothedTx * 0.5 + tx * 0.5;
            smoothedTy = smoothedTy * 0.5 + ty * 0.5;

            double distance = calculateDistance(ta);
            calculatedShooterPower = calculateShooterPower(distance);
            lastValidDistance = distance;

            shooter.setPower(calculatedShooterPower);

            // YAW control - faster response
            double yawAdjustment = smoothedTx * YAW_SERVO_GAIN;
            yawPosition = YAW_CENTER + yawAdjustment;
            yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
            turretYaw.setPosition(yawPosition);

            // PITCH control - faster, more aggressive
            double pitchPower = -smoothedTy * TY_GAIN;
            pitchPower = Math.max(-TURRET_PITCH_SPEED, Math.min(TURRET_PITCH_SPEED, pitchPower));

            turretPitch.setPower(pitchPower);
            sleep(250);  // Shorter pitch adjustment time
            turretPitch.setPower(0);

            boolean onTarget = (Math.abs(smoothedTx) < TARGET_TOLERANCE_TX) &&
                    (Math.abs(smoothedTy) < TARGET_TOLERANCE_TY);

            telemetry.addData("TX", "%.2f (target < %.1f)", smoothedTx, TARGET_TOLERANCE_TX);
            telemetry.addData("TY", "%.2f (target < %.1f)", smoothedTy, TARGET_TOLERANCE_TY);
            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Power", "%.3f", calculatedShooterPower);
            telemetry.addData("On Target", onTarget ? "YES" : "NO");
            telemetry.addData("Stable Count", stableCount);
            telemetry.update();

            if (onTarget && validReadings >= 2) {
                stableCount++;
            } else {
                stableCount = Math.max(0, stableCount - 1);  // Decay slower
            }

            if (stableCount >= 3) {  // Lock faster - only 3 stable readings needed
                turretPitch.setPower(0);
                telemetry.addData("LOCKED!", "Target acquired");
                telemetry.update();
                return true;
            }

            sleep(30);  // Faster loop
        }

        turretPitch.setPower(0);
        telemetry.addData("Timeout", "Failed to lock in time");
        telemetry.update();
        return false;
    }

    public void aimTurretYaw(double yawAngle) {
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawAngle));
        turretYaw.setPosition(yawPosition);
    }

    public void centerTurret() {
        yawPosition = YAW_CENTER;
        turretYaw.setPosition(YAW_CENTER);
    }

    public void spinUpShooter() {
        shooter.setPower(calculatedShooterPower);
        sleep(MOTOR_SPINUP_TIME);
    }

    public void stopShooter() {
        shooter.setPower(0);
    }

    public void shootBallsOptimized() {
        shootBallsWithPattern(null);
    }

    public void shootBallsWithPattern(String[] targetPattern) {
        int[] shootOrder = (targetPattern == null) ? new int[]{0, 1, 2} : calculateShootOrder(targetPattern);

        telemetry.addData("Shoot Order", "[%d, %d, %d]", shootOrder[0], shootOrder[1], shootOrder[2]);
        telemetry.update();
        sleep(500);

        // === BALL 1 ===
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

        // === BALL 2 ===
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

        // === BALL 3 ===
        telemetry.addData("Shooting", "Ball 3 from slot %d", shootOrder[2]);
        telemetry.update();

        int slot3 = shootOrder[2];
        double ball3Power = calculatedShooterPower * BALL_3_POWER_MULTIPLIER;
        shooter.setPower(ball3Power);

        sortWheel.setPosition(getShootPositionForSlot(slot3));
        sleep(SERVO_MOVE_DELAY);

        kicker.setPosition(KICKER_OPEN);
        sleep(SHOOT_DELAY + 100);  // Extra 100ms for ball 3

        kicker.setPosition(KICKER_CLOSED);
        sleep(KICKER_CLOSE_DELAY);

        telemetry.addData("Complete", "All 3 balls fired");
        telemetry.update();
    }

    private double getShootPositionForSlot(int slot) {
        switch (slot) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
        }
    }

    public void driveForward(double inches, double power) {
        int ms = (int)((inches / INCHES_PER_SECOND) * 1000 / power);
        mecanum(power, 0, 0);
        sleep(ms);
        stopDrive();
    }

    public void driveBackward(double inches, double power) {
        int ms = (int)((inches / INCHES_PER_SECOND) * 1000 / power);
        mecanum(-power, 0, 0);
        sleep(ms);
        stopDrive();
    }

    public void strafeRight(double inches, double power) {
        int ms = (int)((inches / INCHES_PER_SECOND) * 1000 / power);
        mecanum(0, power, 0);
        sleep(ms);
        stopDrive();
    }

    public void strafeLeft(double inches, double power) {
        int ms = (int)((inches / INCHES_PER_SECOND) * 1000 / power);
        mecanum(0, -power, 0);
        sleep(ms);
        stopDrive();
    }

    public void turnRight(double degrees, double power) {
        int ms = (int)((degrees / DEGREES_PER_SECOND) * 1000 / power);
        mecanum(0, 0, power);
        sleep(ms);
        stopDrive();
    }

    public void turnLeft(double degrees, double power) {
        int ms = (int)((degrees / DEGREES_PER_SECOND) * 1000 / power);
        mecanum(0, 0, -power);
        sleep(ms);
        stopDrive();
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

        double max = Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
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