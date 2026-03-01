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

@TeleOp(name="TheRobot_AutoAim", group="Competition")
public class TheRobot_AutoAimJay extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system
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
    private static final double IDLE_SHOOTER_POWER = 0.65;
    private static final double VELOCITY_TOLERANCE = 0.02;
    private static final int MAX_VELOCITY_CHECK_TIME = 300;
    private static final int VELOCITY_SAMPLE_COUNT = 3;

    // Distance calculation constants
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 300.0;

    // Storage positions
    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.375;
    private static final double SLOT_2_POSITION = 0.78;

    // Shooting positions
    private static final double SHOOT_POSITION_0 = 0.57;
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;

    // Kicker positions
    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    // Turret yaw servo tuning
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double MANUAL_YAW_STEP = 0.01;

    // Auto-aim constants
    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.5;
    private static final double TX_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 1.0;

    // Adaptive power ramping
    private static final double STUCK_TIME_THRESHOLD = 500;
    private static final double POWER_RAMP_RATE = 0.1;
    private double currentPitchPower = TURRET_PITCH_MIN_SPEED;
    private ElapsedTime stuckTimer = new ElapsedTime();
    private double lastTxError = 0;
    private boolean wasStuck = false;

    // Intake constants
    private static final int SHOOT_DELAY = 375;
    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 150;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.6;

    // Servo specs
    private static final double TORQUE_SERVO_SEC_PER_60DEG = 0.20;
    private static final double SPEED_SERVO_SEC_PER_60DEG = 0.09;
    private static final double SERVO_RANGE_DEGREES = 300.0;
    private static final double SERVO_SAFETY_MARGIN = 1.2;

    // Ball storage
    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;

    // Target pattern
    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    // State variables
    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean isShooting = false;
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true;
    private boolean inCooldown = false;
    private boolean shooterEnabled = false;
    private boolean rollerRunning = false;

    // Last valid values for tracking loss
    private double lastValidYawPos = YAW_CENTER;
    private double lastValidShooterPower = 0.0;
    private double lastValidTX = 0.0;

    // Toggle states
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;
    private boolean lastDpadDownState = false;

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();

    // Debug
    private String distanceMethod = "NONE";
    private double rawX = 0, rawY = 0, rawZ = 0;
    private double rawHorizontal = 0;
    private double rawAngleDist = 0;

    // Filtering
    private double filteredTX = 0;
    private double filteredDist = 0;

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
                sleep(100);
            }
            lastXState = gamepad1.x;

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

            // Display telemetry
            if (shooterEnabled) {
                double currentVel = shooter.getVelocity();
                telemetry.addData("Current Velocity", "%.0f tps", currentVel);
                if (targetShooterVelocity > 0) {
                    double error = Math.abs(currentVel - targetShooterVelocity) / targetShooterVelocity * 100;
                    telemetry.addData("Velocity Error", "%.1f%%", error);
                }
            }

            telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("Roller", rollerRunning ? "ON" : "OFF");
            telemetry.addData("Shooter Power", "%.3f", calculatedShooterPower);
            telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
            telemetry.addData("Filtered TX", "%.2f°", filteredTX);
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

        // Shooter system
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_CLOSED);

        // Sort wheel
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

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize ball slots
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

        // Check for valid tracking
        if (result == null || !result.isValid()) {
            handleTrackingLoss();
            distanceMethod = "NO_DATA";
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            handleTrackingLoss();
            distanceMethod = "NO_TAG";
            return;
        }

        // Find GOAL tag (ID 20 blue, ID 24 red)
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

        // Filter TX
        filteredTX = kalman(filteredTX, tx, 0.7);

        // Calculate distance
        double estimatedDistance = calculateDistance(target, ty);
        filteredDist = kalman(filteredDist, estimatedDistance, 0.65);
        lastValidDistance = filteredDist;

        // Calculate shooter power
        calculatedShooterPower = calculateShooterPower(filteredDist);

        // Calculate and set yaw angle
        double targetYawAngle = calculateYawAngle(filteredDist);
        yawPosition = angleToServoPosition(targetYawAngle);
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);
        lastValidYawPos = yawPosition;

        // Pitch motor control with adaptive ramping
        double pitchPower = 0;
        if (Math.abs(filteredTX) > PITCH_DEADZONE) {
            boolean isStuck = Math.abs(filteredTX - lastTxError) < 0.5;

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

            double proportionalPower = filteredTX * TX_GAIN;
            double powerMagnitude = Math.min(currentPitchPower, Math.abs(proportionalPower));
            pitchPower = (filteredTX > 0) ? powerMagnitude : -powerMagnitude;
            lastTxError = filteredTX;
        } else {
            currentPitchPower = TURRET_PITCH_MIN_SPEED;
            wasStuck = false;
        }

        turretPitch.setPower(pitchPower);
        sleep(150);
        turretPitch.setPower(0);

        // Update shooter
        lastValidShooterPower = calculatedShooterPower;
        if (shooterEnabled && !isShooting) {
            shooter.setPower(calculatedShooterPower);
        }

        telemetry.addData("tx", "%.2f°", tx);
        telemetry.addData("ty", "%.2f°", ty);
        telemetry.addData("Yaw Angle", "%.2f°", targetYawAngle);
        telemetry.addData("Pitch Power", "%.2f (max: %.2f)", pitchPower, currentPitchPower);
    }

    private void handleTrackingLoss() {
        telemetry.addLine("⚠ Lost tracking - maintaining last position");
        turretYaw.setPosition(lastValidYawPos);
        turretPitch.setPower(0);

        if (shooterEnabled && !isShooting) {
            shooter.setPower(lastValidShooterPower);
        } else if (!isShooting) {
            shooter.setPower(0);
        }
    }

    private double calculateDistance(LLResultTypes.FiducialResult target, double ty) {
        Pose3D cameraPose = target.getCameraPoseTargetSpace();

        if (cameraPose != null) {
            Position pos = cameraPose.getPosition();

            rawX = pos.x * 39.3701;
            rawY = pos.y * 39.3701;
            rawZ = pos.z * 39.3701;

            rawHorizontal = Math.sqrt(rawZ * rawZ + rawX * rawX);
            distanceMethod = String.format("3D_POSE(x:%.1f y:%.1f z:%.1f)", rawX, rawY, rawZ);

            return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, rawHorizontal));
        }

        return calculateDistanceFromAngle(ty);
    }

    private double calculateDistanceFromAngle(double ty) {
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;
        double angleRadians = Math.toRadians(totalAngle);
        rawAngleDist = HEIGHT_DIFF / Math.tan(angleRadians);
        distanceMethod = String.format("ANGLE(ty:%.2f totalAngle:%.2f)", ty, totalAngle);
        return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, rawAngleDist));
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

    private double calculateShooterPower(double distanceInches) {
        double power = 0.52 + ((Math.pow(distanceInches, distanceInches * 0.004)) / 100);
        return Math.max(0.50, Math.min(0.85, power));
    }

    private void manualTurretControl() {
        if (gamepad1.left_bumper) {
            yawPosition -= MANUAL_YAW_STEP;
        } else if (gamepad1.right_bumper) {
            yawPosition += MANUAL_YAW_STEP;
        }

        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) {
            pitchPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger > 0.1 && !intakeEnabled) {
            pitchPower = gamepad1.right_trigger;
        }

        turretPitch.setPower(pitchPower);

        if (shooterEnabled && !isShooting) {
            shooter.setPower(IDLE_SHOOTER_POWER);
        }

        distanceMethod = "MANUAL";
    }

    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS || inCooldown) {
            if (inCooldown && cooldownTimer.milliseconds() > DETECTION_COOLDOWN) {
                inCooldown = false;
            }
            return;
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
                telemetry.update();

                if (ballCount >= MAX_BALLS) {
                    intakeEnabled = false;
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
            case 0: sortWheel.setPosition(SLOT_0_POSITION); break;
            case 1: sortWheel.setPosition(SLOT_1_POSITION); break;
            case 2: sortWheel.setPosition(SLOT_2_POSITION); break;
        }
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

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                telemetry.addData("Success", "Pattern loaded from Tag " + id);
                telemetry.addData("Pattern", getPatternString());
                telemetry.update();
                sleep(1500);
                return;
            }
        }

        telemetry.addData("Error", "No pattern tags (21, 22, 23) found");
        telemetry.update();
        sleep(750);
    }

    private void setTargetPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21: targetPattern = new String[]{"Green", "Purple", "Purple"}; break;
            case 22: targetPattern = new String[]{"Purple", "Green", "Purple"}; break;
            case 23: targetPattern = new String[]{"Purple", "Purple", "Green"}; break;
            default: targetPattern = null;
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
        if (isShooting) return;

        isShooting = true;
        roller.setPower(0);
        rollerRunning = false;
        intakeEnabled = false;
        inCooldown = true;

        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

        int[] shootOrder = new int[3];

        if (targetPattern == null || !motifDetected) {
            shootOrder[0] = 0;
            shootOrder[1] = 1;
            shootOrder[2] = 2;
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
                    shootOrder[0] = 0;
                    shootOrder[1] = 1;
                    shootOrder[2] = 2;
                    break;
                }

                shootOrder[patternIndex] = slotWithRequiredBall;
            }
        }

        shooter.setPower(calculatedShooterPower);

        int firstSlot = shootOrder[0];
        double firstShootPosition = getShootPositionForSlot(firstSlot);
        sortWheel.setPosition(firstShootPosition);

        int firstMoveDelay = calculateServoDelay(SLOT_0_POSITION, firstShootPosition, false);
        sleep(Math.max(900, firstMoveDelay));

        double initialVelocity = getAverageVelocity();
        targetShooterVelocity = initialVelocity * 0.935;
        shooter.setPower(calculatedShooterPower * 0.9675);
        sleep(300);

        telemetry.addData("Initial Velocity", "%.0f tps", initialVelocity);
        telemetry.addData("Target Velocity", "%.0f tps", targetShooterVelocity);
        telemetry.update();

        double currentPosition = firstShootPosition;

        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            int slotToShoot = shootOrder[shootIndex];
            String ballColor = ballSlots[slotToShoot];

            if (ballColor == null) continue;

            if (shootIndex > 0) {
                double shootPosition = getShootPositionForSlot(slotToShoot);
                sortWheel.setPosition(shootPosition);

                int moveDelay = calculateServoDelay(currentPosition, shootPosition, false);
                sleep(moveDelay + 150);
                currentPosition = shootPosition;

                boolean velocityReady = waitForVelocityStabilization();
                if (!velocityReady) {
                    telemetry.addData("Warning", "Velocity low for ball " + (shootIndex + 1));
                    telemetry.update();
                }
            }

            kicker.setPosition(KICKER_OPEN);
            sleep(calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100);
            sleep(SHOOT_DELAY);
            kicker.setPosition(KICKER_CLOSED);
            sleep(calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true) + 130);

            ballSlots[slotToShoot] = null;
            ballCount--;

            telemetry.addData("Ball " + (shootIndex + 1), "FIRED");
            telemetry.addData("Velocity", "%.0f tps", shooter.getVelocity());
            telemetry.update();
        }

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
     */
    private double getAverageVelocity() {
        double sum = 0;
        for (int i = 0; i < VELOCITY_SAMPLE_COUNT; i++) {
            sum += Math.abs(shooter.getVelocity());
            if (i < VELOCITY_SAMPLE_COUNT - 1) {
                sleep(20);
            }
        }
        return sum / VELOCITY_SAMPLE_COUNT;
    }

    /**
     * Wait for shooter velocity to stabilize within tolerance
     */
    private boolean waitForVelocityStabilization() {
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();

        while (velocityTimer.milliseconds() < MAX_VELOCITY_CHECK_TIME) {
            double currentVelocity = getAverageVelocity();
            double velocityError = Math.abs(currentVelocity - targetShooterVelocity) / targetShooterVelocity;

            if (velocityError <= VELOCITY_TOLERANCE) {
                return true;
            }

            sleep(50);
        }

        return false;
    }

    private double kalman(double oldVal, double newVal, double k) {
        return (oldVal * (1 - k)) + (newVal * k);
    }
}