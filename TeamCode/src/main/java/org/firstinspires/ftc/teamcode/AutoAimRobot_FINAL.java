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
import java.util.List;

@TeleOp(name="TheRobot", group="Competition")
public class AutoAimRobot_FINAL extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system - using DcMotorEx for velocity control
    private DcMotorEx shooter;
    private DcMotor roller;
    private Servo kicker;

    // Sort wheel / Spindexer
    private ServoImplEx sortWheel;
    private ColorSensor cs;

    // Turret control
    private Servo turretYaw;
    private DcMotor turretPitch;

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // Shooter velocity control
    private double targetShooterVelocity = 0;
    private double calculatedShooterPower = 0.65;
    private double lastValidDistance = 72.0;
    private double lastRawDistance = 72.0;
    private static final double IDLE_SHOOTER_POWER = 0.65;
    private static final double VELOCITY_TOLERANCE = 0.03; // 3% margin of error
    private static final int MAX_VELOCITY_CHECK_TIME = 500; // Max ms to wait for velocity stabilization
    private static final int VELOCITY_SAMPLE_COUNT = 3; // Number of samples to average

    // ---------- DISTANCE CALCULATION CONSTANTS ----------
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double DISTANCE_CALIBRATION_OFFSET = 0;
    private static final double POSE_DISTANCE_OFFSET = 0;

    // ---------- STORAGE POSITIONS ----------
    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.375;
    private static final double SLOT_2_POSITION = 0.78;

    // ---------- SHOOTING POSITIONS ----------
    private static final double SHOOT_POSITION_0 = 0.57;
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;

    // ---------- KICKER POSITIONS ----------
    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    // ---------- TURRET YAW SERVO TUNING ----------
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double YAW_SERVO_GAIN = 0.01;
    private static final double MANUAL_YAW_STEP = 0.01;

    // ---------- AUTO-AIM CONSTANTS ----------
    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double TY_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 1.0;

    // Adaptive power ramping
    private static final double STUCK_TIME_THRESHOLD = 500;
    private static final double POWER_RAMP_RATE = 0.1;
    private double currentPitchPower = TURRET_PITCH_MIN_SPEED;
    private ElapsedTime stuckTimer = new ElapsedTime();
    private double lastTxError = 0;
    private boolean wasStuck = false;

    // ---------- INTAKE CONSTANTS ----------
    private static final int SHOOT_DELAY = 375;
    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 150;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.45;

    // ---------- GOBILDA SERVO SPECS ----------
    private static final double TORQUE_SERVO_SEC_PER_60DEG = 0.20;
    private static final double SPEED_SERVO_SEC_PER_60DEG = 0.09;
    private static final double SERVO_RANGE_DEGREES = 300.0;
    private static final double SERVO_SAFETY_MARGIN = 1.2;

    // ---------- BALL STORAGE ----------
    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;

    // ---------- TARGET PATTERN ----------
    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    // ---------- STATE VARIABLES ----------
    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean isShooting = false;
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true;
    private boolean inCooldown = false;
    private boolean shooterEnabled = false;

    // ---------- TOGGLE STATES ----------
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;
    private boolean lastDpadDownState = false;

    // ---------- TIMERS ----------
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();

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

            // Toggle auto-aim with A button
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
                telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
                telemetry.update();
            }
            lastAState = gamepad1.a;

            // Auto-aim or manual turret control
            if (autoAimEnabled) {
                autoAim();
            } else {
                manualTurretControl();
            }

            // Control intake motor with right trigger
            controlRoller();

            // Continuously monitor color sensor and auto-rotate
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            // Scan AprilTag pattern with Y button
            if (gamepad1.y && !lastYState && !isShooting) {
                scanAprilTagPattern();
                sleep(225);
            }
            lastYState = gamepad1.y;

            // Toggle Shooter Motor with DPad Down
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooterEnabled = !shooterEnabled;
                if (shooterEnabled) {
                    shooter.setPower(IDLE_SHOOTER_POWER);
                    telemetry.addData("Shooter", "ENABLED at idle %.2f power", IDLE_SHOOTER_POWER);
                } else if (!isShooting) {
                    shooter.setPower(0);
                    targetShooterVelocity = 0;
                    telemetry.addData("Shooter", "DISABLED");
                }
                telemetry.update();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // Shoot sequence with B button
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState && !isShooting) {
                if (ballCount < MAX_BALLS) {
                    telemetry.addData("Error", "Not enough balls! Have " + ballCount + ", need " + MAX_BALLS);
                    telemetry.update();
                    sleep(750);
                } else {
                    executeShootSequence();
                }
            }
            lastBState = currentBState;

            // Reset/Clear all balls with X button
            boolean currentXState = gamepad1.x;
            if (currentXState && !lastXState && !isShooting) {
                resetBalls();
            }
            lastXState = currentXState;

            // Display current shooter velocity if enabled
            if (shooterEnabled) {
                double currentVel = shooter.getVelocity();
                telemetry.addData("Current Velocity", "%.0f tps", currentVel);
                if (targetShooterVelocity > 0) {
                    double error = Math.abs(currentVel - targetShooterVelocity) / targetShooterVelocity * 100;
                    telemetry.addData("Velocity Error", "%.1f%%", error);
                }
            }

            telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
            telemetry.addData("Distance", "%.1f in", lastValidDistance);
            telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);
            telemetry.update();
        }
    }

    private void initHardware() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Shooter system - using DcMotorEx for velocity control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_CLOSED);

        // Sort wheel with extended PWM range
        sortWheel = hardwareMap.get(ServoImplEx.class, "sortWheel");
        sortWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        sortWheel.setPosition(SLOT_0_POSITION);

        cs = hardwareMap.get(ColorSensor.class, "cs");

        // Turret control
        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretYaw.setPosition(YAW_CENTER);

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize ball slots as empty
        ballCount = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
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

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            if (shooterEnabled && !isShooting) {
                shooter.setPower(IDLE_SHOOTER_POWER);
            }
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            if (shooterEnabled && !isShooting) {
                shooter.setPower(IDLE_SHOOTER_POWER);
            }
            return;
        }

        // Find the GOAL tag (ID 20 for blue, ID 24 for red)
        LLResultTypes.FiducialResult target = fiducials.get(0);
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = (int) f.getFiducialId();
            if (id == 20 || id == 24) {
                target = f;
                break;
            }
        }

        double tx = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();
        double ta = target.getTargetArea();

        // Distance and shooter power calculation using 3D pose
        double estimatedDistance = calculate3DPoseDistance(target);
        calculatedShooterPower = calculateShooterPower(estimatedDistance);
        lastValidDistance = estimatedDistance;

        // YAW SERVO CONTROL (up/down) - Uses ty (vertical offset)
        if (Math.abs(ty) > 2.0) {
            yawPosition += ty * YAW_SERVO_GAIN;
            yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
            turretYaw.setPosition(yawPosition);
        }

        // PITCH MOTOR CONTROL (left/right) - Uses tx with adaptive power ramping
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
        sleep(150);
        turretPitch.setPower(0);

        // Update shooter velocity based on calculated power
        if (shooterEnabled && !isShooting) {
            shooter.setPower(calculatedShooterPower);
        }

        telemetry.addData("tx", "%.2f", tx);
        telemetry.addData("ty", "%.2f", ty);
        telemetry.addData("Pitch Power", "%.2f (max: %.2f)", pitchPower, currentPitchPower);
        telemetry.addData("Yaw Pos", "%.3f", yawPosition);
    }

    private double calculate3DPoseDistance(LLResultTypes.FiducialResult target) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D cameraPose = target.getCameraPoseTargetSpace();

        if (cameraPose == null) {
            return calculateDistanceFromAngle(target.getTargetYDegrees());
        }

        double x = cameraPose.getPosition().x * 39.3701;
        double y = cameraPose.getPosition().y * 39.3701;
        double z = cameraPose.getPosition().z * 39.3701;

        double horizontalDistance = Math.sqrt(x*x + y*y);
        horizontalDistance += POSE_DISTANCE_OFFSET;

        lastRawDistance = Math.sqrt(x*x + y*y + z*z);

        double clampedDistance = Math.max(14.0, Math.min(144.0, horizontalDistance));
        return clampedDistance;
    }

    private double calculateDistanceFromAngle(double ty) {
        double angleToTarget = CAMERA_MOUNT_ANGLE + ty;
        double angleRadians = Math.toRadians(angleToTarget);
        double horizontalDistance = HEIGHT_DIFF / Math.tan(angleRadians);
        horizontalDistance += DISTANCE_CALIBRATION_OFFSET;
        lastRawDistance = horizontalDistance;
        return Math.max(14.0, Math.min(144.0, horizontalDistance));
    }

    private double calculateShooterPower(double estimatedDistance) {
        double minPower = 0.57;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 144;

        double power = + minPower + (estimatedDistance - minDist) * (maxPower - minPower) / (maxDist - minDist);
        return Math.max(minPower, Math.min(maxPower, power));
    }

    private void manualTurretControl() {
        // Bumpers control yaw
        if (gamepad1.left_bumper) {
            yawPosition -= MANUAL_YAW_STEP;
        } else if (gamepad1.right_bumper) {
            yawPosition += MANUAL_YAW_STEP;
        }

        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // Triggers control pitch
        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) {
            pitchPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger > 0.1 && !intakeEnabled) {
            pitchPower = gamepad1.right_trigger;
        }

        turretPitch.setPower(pitchPower);

        // Maintain idle velocity when shooter enabled but not auto-aiming
        if (shooterEnabled && !isShooting) {
            shooter.setPower(IDLE_SHOOTER_POWER);
        }
    }

    private void controlRoller() {
        if (gamepad1.right_trigger > 0.1 && intakeEnabled && !isShooting) {
            roller.setPower(INTAKE_POWER);
        } else {
            roller.setPower(0);
        }
    }

    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS) {
            return;
        }

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

                telemetry.addData("Ball Detected", detectedColor + " in Slot " + currentSlot);
                telemetry.addData("Ball Count", ballCount + "/" + MAX_BALLS);
                telemetry.update();

                if (ballCount >= MAX_BALLS) {
                    intakeEnabled = false;
                    telemetry.addData("Status", "ALL BALLS LOADED");
                    telemetry.update();
                    sleep(375);
                }
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
        } else if ((red > green && blue > green && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))
                || (red + blue > green * 2 && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))) {
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

    private void resetBalls() {
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
        ballCount = 0;
        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        currentSlot = 0;
        sortWheel.setPosition(SLOT_0_POSITION);

        telemetry.addData("Status", "All balls cleared");
        telemetry.update();
        sleep(750);
    }

    private void scanAprilTagPattern() {
        telemetry.addData("Status", "Scanning for AprilTag pattern...");
        telemetry.update();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No valid Limelight data");
            telemetry.update();
            sleep(750);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Error", "No AprilTags detected");
            telemetry.update();
            sleep(750);
            return;
        }

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                target = fiducial;
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                break;
            }
        }

        if (target == null) {
            telemetry.addData("Error", "No pattern tags (21, 22, 23) found");
            telemetry.update();
            sleep(750);
            return;
        }

        telemetry.addData("Success", "Pattern loaded from Tag " + detectedAprilTagId);
        telemetry.addData("Pattern", getPatternString());
        telemetry.update();
        sleep(1500);
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

    private int calculateServoDelay(double startPosition, double endPosition, boolean isSpeedMode) {
        double positionDelta = Math.abs(endPosition - startPosition);
        double degreesDelta = positionDelta * SERVO_RANGE_DEGREES;
        double secPer60Deg = isSpeedMode ? SPEED_SERVO_SEC_PER_60DEG : TORQUE_SERVO_SEC_PER_60DEG;
        double timeSeconds = (degreesDelta / 60.0) * secPer60Deg;
        int delayMs = (int) (timeSeconds * 1000 * SERVO_SAFETY_MARGIN);
        return Math.max(50, delayMs);
    }

    private void executeShootSequence() {
        if (isShooting) {
            return;
        }

        isShooting = true;
        roller.setPower(0);
        intakeEnabled = false;
        inCooldown = true;

        // Calculate shoot order
        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

        int[] shootOrder = new int[3];
        boolean canMatchPattern = true;

        if (targetPattern == null || !motifDetected) {
            shootOrder[0] = 0;
            shootOrder[1] = 1;
            shootOrder[2] = 2;
            canMatchPattern = false;
        } else {
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
                    canMatchPattern = false;
                    shootOrder[0] = 0;
                    shootOrder[1] = 1;
                    shootOrder[2] = 2;
                    break;
                }

                shootOrder[patternIndex] = slotWithRequiredBall;
            }
        }

        // Start shooter and establish target velocity
        shooter.setPower(calculatedShooterPower);

        // Move to first position while shooter spins up
        int firstSlot = shootOrder[0];
        double firstShootPosition = getShootPositionForSlot(firstSlot);
        double previousPosition = SLOT_0_POSITION;
        sortWheel.setPosition(firstShootPosition);

        int firstMoveDelay = calculateServoDelay(previousPosition, firstShootPosition, false);
        sleep(Math.max(900, firstMoveDelay)); // Original timing restored

        // Capture target velocity ONCE after full spin-up - this is the reference for all 3 shots
        targetShooterVelocity = getAverageVelocity();

        telemetry.addData("Target Velocity", "%.0f tps", targetShooterVelocity);
        telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
        telemetry.update();

        // Shoot all 3 balls with velocity verification
        double currentPosition = firstShootPosition;

        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            int slotToShoot = shootOrder[shootIndex];
            String ballColor = ballSlots[slotToShoot];

            if (ballColor == null) {
                continue;
            }

            // Move to position for shots 2 and 3
            if (shootIndex > 0) {
                double shootPosition = getShootPositionForSlot(slotToShoot);
                sortWheel.setPosition(shootPosition);

                int moveDelay = calculateServoDelay(currentPosition, shootPosition, false);
                sleep(moveDelay + 150);

                currentPosition = shootPosition;

                // CRITICAL: Verify velocity before shooting
                boolean velocityReady = waitForVelocityStabilization();

                if (!velocityReady) {
                    // Velocity dropped - this means shooter may have slowed down
                    telemetry.addData("Warning", "Velocity low for ball " + (shootIndex + 1));
                    telemetry.addData("Current", "%.0f tps (target: %.0f)", shooter.getVelocity(), targetShooterVelocity);
                    telemetry.update();

                    // Don't boost power - just wait a bit more for recovery
                    sleep(200);

                } else {
                    telemetry.addData("Ball " + (shootIndex + 1), "Velocity OK");
                    telemetry.update();
                }
            }

            // Fire the ball
            kicker.setPosition(KICKER_OPEN);
            int kickerOpenDelay = calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true);
            sleep(kickerOpenDelay + 100);

            sleep(SHOOT_DELAY);

            kicker.setPosition(KICKER_CLOSED);
            int kickerCloseDelay = calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true);
            sleep(kickerCloseDelay + 130);

            ballSlots[slotToShoot] = null;
            ballCount--;

            // Brief telemetry update
            telemetry.addData("Ball " + (shootIndex + 1), "FIRED");
            telemetry.addData("Velocity", "%.0f tps", shooter.getVelocity());
            telemetry.update();
        }

        // Cleanup
        shooter.setPower(0);
        shooterEnabled = false;
        targetShooterVelocity = 0;

        sortWheel.setPosition(SLOT_0_POSITION);
        currentSlot = 0;

        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        isShooting = false;

        telemetry.addData("Complete", "3 balls fired with velocity control");
        telemetry.update();
        sleep(1000);
    }

    private double getShootPositionForSlot(int slotNumber) {
        switch (slotNumber) {
            case 0:
                return SHOOT_POSITION_0;
            case 1:
                return SHOOT_POSITION_1;
            case 2:
                return SHOOT_POSITION_2;
            default:
                return SHOOT_POSITION_0;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s, %s, %s]", targetPattern[0], targetPattern[1], targetPattern[2]);
    }

    /**
     * Get averaged velocity reading to reduce noise
     * @return Average velocity over multiple samples
     */
    private double getAverageVelocity() {
        double sum = 0;
        for (int i = 0; i < VELOCITY_SAMPLE_COUNT; i++) {
            sum += Math.abs(shooter.getVelocity());
            if (i < VELOCITY_SAMPLE_COUNT - 1) {
                sleep(20); // Small delay between samples
            }
        }
        return sum / VELOCITY_SAMPLE_COUNT;
    }

    /**
     * Wait for shooter velocity to stabilize within tolerance
     * @return true if velocity stabilized within time limit, false otherwise
     */
    private boolean waitForVelocityStabilization() {
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();

        while (velocityTimer.milliseconds() < MAX_VELOCITY_CHECK_TIME) {
            double currentVelocity = getAverageVelocity();
            double velocityError = Math.abs(currentVelocity - targetShooterVelocity) / targetShooterVelocity;

            if (velocityError <= VELOCITY_TOLERANCE) {
                // Velocity is within 3% - good to shoot
                return true;
            }

            // Small delay between checks
            sleep(50);
        }

        // Timed out - velocity didn't stabilize
        return false;
    }

    /**
     * Maintain idle shooter velocity when enabled but not actively aiming/shooting
     */
    private void maintainIdleVelocity() {
        if (targetShooterVelocity <= 0) {
            // No baseline velocity established yet
            shooter.setPower(IDLE_SHOOTER_POWER);
            return;
        }

        // Check current velocity and adjust power if needed
        double currentVelocity = shooter.getVelocity();
        double velocityError = (currentVelocity - targetShooterVelocity) / targetShooterVelocity;

        // Simple proportional control to maintain velocity
        if (Math.abs(velocityError) > VELOCITY_TOLERANCE) {
            double powerAdjustment = velocityError * -0.1; // Negative because we want inverse response
            double newPower = shooter.getPower() + powerAdjustment;
            newPower = Math.max(0.5, Math.min(0.85, newPower)); // Clamp to safe range
            shooter.setPower(newPower);
        }
    }
}