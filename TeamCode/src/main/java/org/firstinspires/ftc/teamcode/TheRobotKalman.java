// VERSION 5 + KALMAN FILTERING (1/21/26) by Aditya and Shreehan - Velocity control with Kalman filtering
// Velocity control (Aditya) and Kalman Filtering Implementation (Shreehan)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.List;

@TeleOp(name="TheRobot (Kalman)", group="Competition")
public class TheRobotKalman extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor roller;
    private Servo kicker;
    private ServoImplEx sortWheel;
    private ColorSensor cs;
    private Servo turretYaw;
    private DcMotor turretPitch;
    private Limelight3A limelight;


    private static final double TICKS_PER_REV = 537.6;
    private static final double MAX_TICKS_PER_SEC = 2800;
    private static final double IDLE_RPM = 180;

    // Distance Kalman Filter
    private double filteredDistance = 72.0;
    private double distanceUncertainty = 10.0;
    private static final double DISTANCE_PROCESS_NOISE = 0.5;
    private static final double DISTANCE_MEASUREMENT_NOISE = 2.0;

    // tx filer
    private double filteredTX = 0.0;
    private double txUncertainty = 1.0;
    private static final double TX_PROCESS_NOISE = 0.1;
    private static final double TX_MEASUREMENT_NOISE = 0.5;

    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double ANGLE_DISTANCE_OFFSET = 0.0;
    private static final double POSE_DISTANCE_OFFSET = 0.0;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 300.0;


    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.375;
    private static final double SLOT_2_POSITION = 0.78;

    private static final double SHOOT_POSITION_0 = 0.57;
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;

    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double MANUAL_YAW_STEP = 0.01;

    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 1.0;

    private static final double STUCK_TIME_THRESHOLD = 500;
    private static final double POWER_RAMP_RATE = 0.1;
    private double currentPitchPower = TURRET_PITCH_MIN_SPEED;
    private ElapsedTime stuckTimer = new ElapsedTime();
    private double lastTxError = 0;
    private boolean wasStuck = false;


    private static final int SHOOT_DELAY = 375;
    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 150;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.6;


    private static final double TORQUE_SERVO_SEC_PER_60DEG = 0.20;
    private static final double SPEED_SERVO_SEC_PER_60DEG = 0.09;
    private static final double SERVO_RANGE_DEGREES = 300.0;
    private static final double SERVO_SAFETY_MARGIN = 1.2;


    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;

    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean isShooting = false;
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true;
    private boolean inCooldown = false;
    private boolean shooterEnabled = false;
    private boolean rollerRunning = false;

    private double targetVelocityTPS = 0;
    private double lastValidDistance = 72.0;

    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;
    private boolean lastDpadDownState = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();

    private String distanceMethod = "NONE";
    private double rawX = 0, rawY = 0, rawZ = 0;
    private double rawHorizontal = 0;
    private double rawAngleDist = 0;


    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            // Toggle shooter enable
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled && !isShooting) {
                    shooter.setVelocity(0);
                    targetVelocityTPS = 0;
                }
            }
            lastDpadDownState = gamepad1.dpad_down;

            // Maintain idle velocity
            if (shooterEnabled && !isShooting) {
                shooter.setVelocity(rpmToTPS(IDLE_RPM));
            }

            // Toggle auto-aim
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastAState = gamepad1.a;

            // Auto-aim or manual control
            if (autoAimEnabled) {
                autoAim();
            } else {
                manualTurretControl();
            }

            // Toggle intake
            if (gamepad1.x && !lastXState && !isShooting) {
                rollerRunning = !rollerRunning;
                roller.setPower(rollerRunning ? INTAKE_POWER : 0);
            }
            lastXState = gamepad1.x;

            // Ball detection
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            // Pattern scanning
            if (gamepad1.y && !lastYState && !isShooting) {
                scanAprilTagPattern();
            }
            lastYState = gamepad1.y;

            // Shooting
            if (gamepad1.b && !lastBState && !isShooting) {
                if (shooterEnabled && ballCount >= MAX_BALLS) {
                    executeShootSequence();
                } else if (!shooterEnabled) {
                    telemetry.addData("Error", "Shooter disabled! Press DPad Down");
                    telemetry.update();
                } else {
                    telemetry.addData("Error", "Need 3 balls! Have: " + ballCount);
                    telemetry.update();
                }
            }
            lastBState = gamepad1.b;

            // Telemetry
            telemetry.addData("Shooter", shooterEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("Target RPM", tpsToRPM(targetVelocityTPS));
            telemetry.addData("Actual RPM", tpsToRPM(shooter.getVelocity()));
            telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
            telemetry.addData("Filtered Dist", "%.1f in", filteredDistance);
            telemetry.addData("Filtered TX", "%.2f°", filteredTX);
            telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);
            telemetry.addData("Pattern", getPatternString());
            telemetry.addData("Roller", rollerRunning ? "ON" : "OFF");
            telemetry.update();
        }
    }


    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_CLOSED);

        sortWheel = hardwareMap.get(ServoImplEx.class, "sortWheel");
        sortWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
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

        for (int i = 0; i < 3; i++) ballSlots[i] = null;
        ballCount = 0;
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



    /**
     * Kalman filter for distance - smooths noisy Limelight readings
     */
    private double filterDistance(double measuredDistance) {
        // Prediction step
        double predictedDistance = filteredDistance;
        double predictedUncertainty = distanceUncertainty + DISTANCE_PROCESS_NOISE;

        // Update step
        double kalmanGain = predictedUncertainty / (predictedUncertainty + DISTANCE_MEASUREMENT_NOISE);
        filteredDistance = predictedDistance + kalmanGain * (measuredDistance - predictedDistance);
        distanceUncertainty = (1 - kalmanGain) * predictedUncertainty;

        return filteredDistance;
    }

    /**
     * Kalman filter for TX - smooths turret aiming
     */
    private double filterTX(double measuredTX) {
        // Prediction step
        double predictedTX = filteredTX;
        double predictedUncertainty = txUncertainty + TX_PROCESS_NOISE;

        // Update step
        double kalmanGain = predictedUncertainty / (predictedUncertainty + TX_MEASUREMENT_NOISE);
        filteredTX = predictedTX + kalmanGain * (measuredTX - predictedTX);
        txUncertainty = (1 - kalmanGain) * predictedUncertainty;

        return filteredTX;
    }

    /* ==================== AUTO-AIM WITH KALMAN FILTERING ==================== */
    private void autoAim() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            distanceMethod = "NO_DATA";
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            distanceMethod = "NO_TAG";
            return;
        }

        // Find goal tag (20 or 24)
        LLResultTypes.FiducialResult target = fiducials.get(0);
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = (int) f.getFiducialId();
            if (id == 20 || id == 24) {
                target = f;
                break;
            }
        }

        double rawTX = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();

        // Apply Kalman filter to TX for smoother turret control
        double tx = filterTX(rawTX);

        // Calculate and filter distance
        double rawDistance = calculateDistance(target, ty);
        double smoothedDistance = filterDistance(rawDistance);
        lastValidDistance = smoothedDistance;

        // Calculate yaw angle and set servo position
        double targetYawAngle = calculateYawAngle(smoothedDistance);
        yawPosition = angleToServoPosition(targetYawAngle);
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // Pitch motor control with adaptive ramping (uses filtered TX)
        double pitchPower = 0;
        if (Math.abs(tx) > PITCH_DEADZONE) {
            boolean isStuck = Math.abs(tx - lastTxError) < 0.5;

            if (isStuck && !wasStuck) {
                stuckTimer.reset();
                wasStuck = true;
            } else if (isStuck && stuckTimer.milliseconds() > STUCK_TIME_THRESHOLD) {
                currentPitchPower = Math.min(TURRET_PITCH_MAX_SPEED,
                        currentPitchPower + POWER_RAMP_RATE);
                stuckTimer.reset();
            } else if (!isStuck) {
                currentPitchPower = TURRET_PITCH_MIN_SPEED;
                wasStuck = false;
            }

            double proportionalPower = tx * TX_GAIN;
            double powerMagnitude = Math.min(currentPitchPower, Math.abs(proportionalPower));
            pitchPower = (tx > 0) ? powerMagnitude : -powerMagnitude;

            lastTxError = tx;
        } else {
            currentPitchPower = TURRET_PITCH_MIN_SPEED;
            wasStuck = false;
        }

        turretPitch.setPower(pitchPower);
    }

    /* ==================== MANUAL TURRET CONTROL ==================== */
    private void manualTurretControl() {
        if (gamepad1.left_bumper) yawPosition -= MANUAL_YAW_STEP;
        if (gamepad1.right_bumper) yawPosition += MANUAL_YAW_STEP;
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) pitchPower = -gamepad1.left_trigger;
        if (gamepad1.right_trigger > 0.1) pitchPower = gamepad1.right_trigger;
        turretPitch.setPower(pitchPower);

        distanceMethod = "MANUAL";
    }

    /* ==================== DISTANCE CALCULATION ==================== */
    private double calculateDistance(LLResultTypes.FiducialResult target, double ty) {
        Pose3D cameraPose = target.getCameraPoseTargetSpace();

        if (cameraPose != null) {
            Position pos = cameraPose.getPosition();
            rawX = pos.x * 39.3701;
            rawY = pos.y * 39.3701;
            rawZ = pos.z * 39.3701;
            rawHorizontal = Math.sqrt(rawZ * rawZ + rawX * rawX);
            double calibratedDistance = rawHorizontal + POSE_DISTANCE_OFFSET;
            distanceMethod = String.format("3D_POSE(x:%.1f y:%.1f z:%.1f)", rawX, rawY, rawZ);
            return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, calibratedDistance));
        }

        return calculateDistanceFromAngle(ty);
    }

    private double calculateDistanceFromAngle(double ty) {
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;
        double angleRadians = Math.toRadians(totalAngle);
        rawAngleDist = HEIGHT_DIFF / Math.tan(angleRadians);
        double calibratedDistance = rawAngleDist + ANGLE_DISTANCE_OFFSET;
        distanceMethod = String.format("ANGLE(ty:%.2f totalAngle:%.2f)", ty, totalAngle);
        return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, calibratedDistance));
    }

    private double calculateYawAngle(double distanceInches) {
        return 1.51 * Math.pow(0.97, distanceInches);
    }

    private double angleToServoPosition(double angle) {
        double maxAngle = calculateYawAngle(MIN_DISTANCE);
        double minAngle = calculateYawAngle(MAX_DISTANCE);
        double normalized = (angle - minAngle) / (maxAngle - minAngle);
        return YAW_MIN + normalized * (YAW_MAX - YAW_MIN);
    }

    /* ==================== BALL DETECTION ==================== */
    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS) return;

        if (inCooldown) {
            if (cooldownTimer.milliseconds() > DETECTION_COOLDOWN) inCooldown = false;
            else return;
        }

        int red = cs.red();
        int green = cs.green();
        int blue = cs.blue();
        boolean ballPresent = red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD;

        if (ballPresent && !ballDetectedInSlot) {
            String color = detectBallColor(red, green, blue);
            if (color != null) {
                ballSlots[currentSlot] = color;
                ballCount++;
                ballDetectedInSlot = true;
                detectionTimer.reset();
                if (ballCount >= MAX_BALLS) intakeEnabled = false;
            }
        }

        if (ballDetectedInSlot && detectionTimer.milliseconds() > AUTO_ROTATE_DELAY) {
            if (ballCount < MAX_BALLS) {
                currentSlot = (currentSlot + 1) % 3;
                switch (currentSlot) {
                    case 0: sortWheel.setPosition(SLOT_0_POSITION); break;
                    case 1: sortWheel.setPosition(SLOT_1_POSITION); break;
                    case 2: sortWheel.setPosition(SLOT_2_POSITION); break;
                }
                inCooldown = true;
                cooldownTimer.reset();
            }
            ballDetectedInSlot = false;
        }
    }

    private String detectBallColor(int red, int green, int blue) {
        if (green > red && green > blue && green > COLOR_THRESHOLD) return "Green";
        else if ((red > green && blue > green && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))
                || (red + blue > green * 2 && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))) return "Purple";
        return null;
    }

    /* ==================== PATTERN SCANNING ==================== */
    private void scanAprilTagPattern() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No Limelight data");
            telemetry.update();
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Error", "No AprilTags found");
            telemetry.update();
            return;
        }

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = (int) f.getFiducialId();
            if (id >= 21 && id <= 23) {
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                telemetry.addData("Success", "Pattern loaded from Tag " + id);
                telemetry.addData("Pattern", getPatternString());
                telemetry.update();
                return;
            }
        }

        telemetry.addData("Error", "No pattern tags (21-23) found");
        telemetry.update();
    }

    private void setTargetPattern(int id) {
        switch (id) {
            case 21: targetPattern = new String[]{"Green","Purple","Purple"}; break;
            case 22: targetPattern = new String[]{"Purple","Green","Purple"}; break;
            case 23: targetPattern = new String[]{"Purple","Purple","Green"}; break;
            default: targetPattern = null;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s,%s,%s]", targetPattern[0], targetPattern[1], targetPattern[2]);
    }

    /* ==================== SHOOTING SEQUENCE (VERSION 3 TIMINGS) ==================== */
    private void executeShootSequence() {
        if (isShooting) return;
        isShooting = true;
        roller.setPower(0);
        rollerRunning = false;
        intakeEnabled = false;

        // Use filtered distance for RPM calculation
        double distance = filteredDistance;
        double targetRPM = calculateShooterRPM(distance);
        targetVelocityTPS = Math.min(rpmToTPS(targetRPM), MAX_TICKS_PER_SEC);

        shooter.setVelocity(targetVelocityTPS);

        int[] shootOrder = calculateShootOrder();
        int firstSlot = shootOrder[0];
        double firstShootPosition = getShootPositionForSlot(firstSlot);
        sortWheel.setPosition(firstShootPosition);

        sleep(600);

        // Shoot all 3 balls with VERSION 3 timing
        for (int i = 0; i < 3; i++) {
            if (!opModeIsActive()) break;

            int slot = shootOrder[i];
            if (ballSlots[slot] == null) continue;

            if (i > 0) {
                double shootPosition = getShootPositionForSlot(slot);
                sortWheel.setPosition(shootPosition);
                sleep(calculateServoDelay(0.5, shootPosition, false) + 150);
            }

            // Fire the ball with calculated delays
            kicker.setPosition(KICKER_OPEN);
            int kickerOpenDelay = calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true);
            sleep(kickerOpenDelay + 100);

            sleep(SHOOT_DELAY);

            kicker.setPosition(KICKER_CLOSED);
            int kickerCloseDelay = calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true);
            sleep(kickerCloseDelay + 130);

            ballSlots[slot] = null;
            ballCount--;

            telemetry.addData("Ball " + (i+1), "FIRED");
            telemetry.addData("Velocity", "%.0f tps (%.0f RPM)", shooter.getVelocity(), tpsToRPM(shooter.getVelocity()));
            telemetry.update();
        }

        // Cleanup
        shooter.setVelocity(0);
        targetVelocityTPS = 0;
        shooterEnabled = false;

        sortWheel.setPosition(SLOT_0_POSITION);
        currentSlot = 0;

        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        isShooting = false;

        telemetry.addData("Complete", "3 balls fired");
        telemetry.update();
    }

    /* ==================== PATTERN MATCHING ==================== */
    private int[] calculateShootOrder() {
        int[] shootOrder = new int[3];

        if (targetPattern == null || !motifDetected) {
            return new int[]{0, 1, 2};
        }

        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

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
                return new int[]{0, 1, 2};
            }

            shootOrder[patternIndex] = slotWithRequiredBall;
        }

        return shootOrder;
    }

    /* ==================== UTILITIES ==================== */

    private double calculateShooterRPM(double distanceInches) {
        return 220 + (distanceInches * 1.8);
    }

    private double getShootPositionForSlot(int slot) {
        switch (slot) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
        }
    }

    private int calculateServoDelay(double startPosition, double endPosition, boolean isSpeedMode) {
        double positionDelta = Math.abs(endPosition - startPosition);
        double degreesDelta = positionDelta * SERVO_RANGE_DEGREES;
        double secPer60Deg = isSpeedMode ? SPEED_SERVO_SEC_PER_60DEG : TORQUE_SERVO_SEC_PER_60DEG;
        double timeSeconds = (degreesDelta / 60.0) * secPer60Deg;
        int delayMs = (int) (timeSeconds * 1000 * SERVO_SAFETY_MARGIN);
        return Math.max(50, delayMs);
    }

    private double rpmToTPS(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double tpsToRPM(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }
}