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

@TeleOp(name="TheRobot", group="Competition")
public class TheRobot extends LinearOpMode {

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
    private double lastValidDistance = 72.0; // inches
    private double lastRawDistance = 72.0; // inches
    private static final double IDLE_SHOOTER_POWER = 0.65;
    private static final double VELOCITY_TOLERANCE = 0.02; // 2% margin of error
    private static final int MAX_VELOCITY_CHECK_TIME = 300; // Max ms to wait for velocity stabilization
    private static final int VELOCITY_SAMPLE_COUNT = 3; // Number of samples to average

    // ---------- DISTANCE CALCULATION CONSTANTS (ALL IN INCHES) ----------
    private static final double CAMERA_HEIGHT = 15.375; // inches off ground
    private static final double TARGET_HEIGHT = 29.5; // inches off ground
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT; // 14.125 inches
    private static final double CAMERA_MOUNT_ANGLE = 8.75; // degrees (positive = tilted up)

    // Calibration offsets - adjust these based on testing
    private static final double ANGLE_DISTANCE_OFFSET = 0.0; // inches to add/subtract from angle-based calculation
    private static final double POSE_DISTANCE_OFFSET = 0.0; // inches to add/subtract from 3D pose calculation

    // Distance limits - TEMPORARILY DISABLED FOR DEBUGGING
    private static final double MIN_DISTANCE = 1.0; // inches (was 14.0)
    private static final double MAX_DISTANCE = 300.0; // inches (was 144.0)

    // Debug variables
    private double rawX = 0, rawY = 0, rawZ = 0;
    private double rawHorizontal = 0;
    private double rawAngleDist = 0;

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
    private static final double INTAKE_POWER = 0.6;

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
    private boolean rollerRunning = false;

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
    private ElapsedTime buttonFeedbackTimer = new ElapsedTime();
    private ElapsedTime autoAimDelayTimer = new ElapsedTime();
    private ElapsedTime patternScanTimer = new ElapsedTime();
    private ElapsedTime statusMessageTimer = new ElapsedTime();

    // ---------- WAITING STATE ----------
    private boolean isWaitingForDelay = false;
    private ElapsedTime waitTimer = new ElapsedTime();
    private int waitDuration = 0;

    // ---------- DEBUG ----------
    private String distanceMethod = "NONE";

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

            // Toggle intake motor with X button
            if (gamepad1.x && !lastXState && !isShooting) {
                rollerRunning = !rollerRunning;
                roller.setPower(rollerRunning ? INTAKE_POWER : 0);
                telemetry.addData("Intake", rollerRunning ? "ON" : "OFF");
                telemetry.update();
                buttonFeedbackTimer.reset();
            }
            lastXState = gamepad1.x;

            // Continuously monitor color sensor and auto-rotate
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            // Scan AprilTag pattern with Y button
            if (gamepad1.y && !lastYState && !isShooting) {
                scanAprilTagPattern();
                patternScanTimer.reset();
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
                    statusMessageTimer.reset();
                } else {
                    executeShootSequence();
                }
            }
            lastBState = currentBState;

            // Display current shooter velocity if enabled
            if (shooterEnabled) {
                double currentVel = shooter.getVelocity();
                telemetry.addData("Current Velocity", "%.0f tps", currentVel);
                if (targetShooterVelocity > 0) {
                    double error = Math.abs(currentVel - targetShooterVelocity) / targetShooterVelocity * 100;
                    telemetry.addData("Velocity Error", "%.1f%%", error);
                }
            }

            telemetry.addData("Roller", rollerRunning ? "ON" : "OFF");
            telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
            telemetry.addData("=== DISTANCE INFO ===", "");
            telemetry.addData("Method", distanceMethod);
            telemetry.addData("Final Distance", "%.1f in", lastValidDistance);
            if (rawHorizontal > 0) {
                telemetry.addData("Raw Horiz", "%.2f in", rawHorizontal);
                telemetry.addData("Raw XYZ", "X:%.1f Y:%.1f Z:%.1f", rawX, rawY, rawZ);
            }
            if (rawAngleDist > 0) {
                telemetry.addData("Raw Angle Dist", "%.2f in", rawAngleDist);
            }
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
            distanceMethod = "NO_DATA";
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            if (shooterEnabled && !isShooting) {
                shooter.setPower(IDLE_SHOOTER_POWER);
            }
            distanceMethod = "NO_TAG";
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

        // Distance calculation - try 3D pose first, fall back to angle-based
        double estimatedDistance = calculateDistance(target, ty);

        // Calculate shooter power using formula: 0.53755 * 1.00257^x
        calculatedShooterPower = calculateShooterPower(estimatedDistance);
        lastValidDistance = estimatedDistance;

        // Calculate yaw angle using formula: 1.51735 * 0.982472^x
        double targetYawAngle = calculateYawAngle(estimatedDistance);

        // Convert angle to servo position
        yawPosition = angleToServoPosition(targetYawAngle);
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

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

        // Use non-blocking delay for pitch motor
        if (autoAimDelayTimer.milliseconds() >= 150) {
            turretPitch.setPower(0);
            autoAimDelayTimer.reset();
        }

        // Update shooter velocity based on calculated power
        if (shooterEnabled && !isShooting) {
            shooter.setPower(calculatedShooterPower);
        }

        telemetry.addData("tx", "%.2f", tx);
        telemetry.addData("ty", "%.2f", ty);
        telemetry.addData("Yaw Angle", "%.2f°", targetYawAngle);
        telemetry.addData("Pitch Power", "%.2f (max: %.2f)", pitchPower, currentPitchPower);
        telemetry.addData("Yaw Pos", "%.3f", yawPosition);
    }

    private double calculateDistance(LLResultTypes.FiducialResult target, double ty) {
        // Try 3D pose first
        Pose3D cameraPose = target.getCameraPoseTargetSpace();

        if (cameraPose != null) {
            Position pos = cameraPose.getPosition();

            // Store raw values in METERS (Limelight's native unit)
            double x_meters = pos.x;
            double y_meters = pos.y;
            double z_meters = pos.z;

            // Convert from meters to inches (1 meter = 39.3701 inches)
            rawX = x_meters * 39.3701;
            rawY = y_meters * 39.3701;
            rawZ = z_meters * 39.3701;

            // Calculate horizontal distance
            rawHorizontal = Math.sqrt(rawZ * rawZ + rawX * rawX);

            // Apply calibration offset
            double calibratedDistance = rawHorizontal + POSE_DISTANCE_OFFSET;

            // Store for debugging
            lastRawDistance = rawHorizontal;
            distanceMethod = String.format("3D_POSE(x:%.1f y:%.1f z:%.1f)", rawX, rawY, rawZ);

            // Clamp to reasonable range
            return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, calibratedDistance));
        }

        // Fall back to angle-based calculation
        return calculateDistanceFromAngle(ty);
    }

    private double calculateDistanceFromAngle(double ty) {
        // Total angle to target = camera mount angle + ty offset
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;

        // Convert to radians
        double angleRadians = Math.toRadians(totalAngle);

        // Calculate horizontal distance using trigonometry
        rawAngleDist = HEIGHT_DIFF / Math.tan(angleRadians);

        // Apply calibration offset
        double calibratedDistance = rawAngleDist + ANGLE_DISTANCE_OFFSET;

        // Store for debugging
        lastRawDistance = rawAngleDist;
        distanceMethod = String.format("ANGLE(ty:%.2f totalAngle:%.2f)", ty, totalAngle);

        // Clamp to reasonable range
        return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, calibratedDistance));
    }

    private double calculateYawAngle(double distanceInches) {
        return 1.51 * Math.pow(0.97, distanceInches);
    }

    private double angleToServoPosition(double angle) {
        // Calculate angle range based on distance extremes
        double maxAngle = calculateYawAngle(MIN_DISTANCE);
        double minAngle = calculateYawAngle(MAX_DISTANCE);

        // Normalize angle to 0-1 range
        double normalized = (angle - minAngle) / (maxAngle - minAngle);

        // Map to servo range
        return YAW_MIN + normalized * (YAW_MAX - YAW_MIN);
    }

    private double calculateShooterPower(double distanceInches) {
        double power = 0.55 + ((Math.pow(distanceInches, distanceInches * 0.004)) / 100);
        // Clamp to safe motor power range
        return Math.max(0.50, Math.min(0.85, power));
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

        distanceMethod = "MANUAL";
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
                    statusMessageTimer.reset();
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
        statusMessageTimer.reset();
    }

    private void scanAprilTagPattern() {
        telemetry.addData("Status", "Scanning for AprilTag pattern...");
        telemetry.update();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No valid Limelight data");
            telemetry.update();
            statusMessageTimer.reset();
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Error", "No AprilTags detected");
            telemetry.update();
            statusMessageTimer.reset();
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
            statusMessageTimer.reset();
            return;
        }

        telemetry.addData("Success", "Pattern loaded from Tag " + detectedAprilTagId);
        telemetry.addData("Pattern", getPatternString());
        telemetry.update();
        statusMessageTimer.reset();
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
        rollerRunning = false;
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
        ElapsedTime spinupTimer = new ElapsedTime();
        while (opModeIsActive() && spinupTimer.milliseconds() < Math.max(900, firstMoveDelay)) {
            // Allow telemetry updates during wait
            telemetry.addData("Status", "Spinning up shooter...");
            telemetry.addData("Time", "%.0f/%.0f ms", spinupTimer.milliseconds(), (double) Math.max(900, firstMoveDelay));
            telemetry.update();
        }

        // Capture initial velocity after spin-up
        double initialVelocity = getAverageVelocity();

        // Target velocity accounting for drop
        targetShooterVelocity = initialVelocity * 0.935;

        // Reduce power to hit the target velocity
        double powerReduction = 1.0 - 0.935;
        shooter.setPower(calculatedShooterPower * (1.0 - powerReduction * 0.5));

        // Let it stabilize at new power
        ElapsedTime stabilizeTimer = new ElapsedTime();
        while (opModeIsActive() && stabilizeTimer.milliseconds() < 300) {
            telemetry.addData("Status", "Stabilizing velocity...");
            telemetry.update();
        }

        telemetry.addData("Initial Velocity", "%.0f tps", initialVelocity);
        telemetry.addData("Target Velocity", "%.0f tps", targetShooterVelocity);
        telemetry.addData("Adjusted Power", "%.3f", shooter.getPower());
        telemetry.update();

        // Shoot all 3 balls with velocity verification
        double currentPosition = firstShootPosition;

        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            if (!opModeIsActive()) break;

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
                ElapsedTime moveTimer = new ElapsedTime();
                while (opModeIsActive() && moveTimer.milliseconds() < moveDelay + 150) {
                    telemetry.addData("Status", "Moving to position " + (shootIndex + 1));
                    telemetry.update();
                }

                currentPosition = shootPosition;

                // Verify velocity before shooting
                boolean velocityReady = waitForVelocityStabilization();

                if (!velocityReady) {
                    telemetry.addData("Warning", "Velocity low for ball " + (shootIndex + 1));
                    telemetry.addData("Current", "%.0f tps (target: %.0f)", shooter.getVelocity(), targetShooterVelocity);
                    telemetry.update();
                    ElapsedTime warningTimer = new ElapsedTime();
                    while (opModeIsActive() && warningTimer.milliseconds() < 200) {
                        // Brief pause
                    }
                } else {
                    telemetry.addData("Ball " + (shootIndex + 1), "Velocity OK");
                    telemetry.update();
                }
            }

            // Fire the ball
            kicker.setPosition(KICKER_OPEN);
            int kickerOpenDelay = calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true);
            ElapsedTime kickOpenTimer = new ElapsedTime();
            while (opModeIsActive() && kickOpenTimer.milliseconds() < kickerOpenDelay + 100) {
                // Wait for kicker to open
            }

            ElapsedTime shootDelayTimer = new ElapsedTime();
            while (opModeIsActive() && shootDelayTimer.milliseconds() < SHOOT_DELAY) {
                // Ball firing delay
            }

            kicker.setPosition(KICKER_CLOSED);
            int kickerCloseDelay = calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true);
            ElapsedTime kickCloseTimer = new ElapsedTime();
            while (opModeIsActive() && kickCloseTimer.milliseconds() < kickerCloseDelay + 130) {
                // Wait for kicker to close
            }

            ballSlots[slotToShoot] = null;
            ballCount--;

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
        statusMessageTimer.reset();
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

    private double getAverageVelocity() {
        double sum = 0;
        for (int i = 0; i < VELOCITY_SAMPLE_COUNT; i++) {
            sum += Math.abs(shooter.getVelocity());
            if (i < VELOCITY_SAMPLE_COUNT - 1) {
                ElapsedTime sampleTimer = new ElapsedTime();
                while (opModeIsActive() && sampleTimer.milliseconds() < 20) {
                    // Brief delay between samples
                }
            }
        }
        return sum / VELOCITY_SAMPLE_COUNT;
    }

    private boolean waitForVelocityStabilization() {
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();

        while (opModeIsActive() && velocityTimer.milliseconds() < MAX_VELOCITY_CHECK_TIME) {
            double currentVelocity = getAverageVelocity();
            double velocityError = Math.abs(currentVelocity - targetShooterVelocity) / targetShooterVelocity;

            if (velocityError <= VELOCITY_TOLERANCE) {
                return true;
            }

            ElapsedTime checkDelayTimer = new ElapsedTime();
            while (opModeIsActive() && checkDelayTimer.milliseconds() < 50) {
                // Brief delay between checks
            }
        }

        return false;
    }
}