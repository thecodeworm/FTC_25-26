package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.List;

@TeleOp(name="VoltageControlled_Robot", group="Competition")
public class VoltageControlledRobot extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system - VOLTAGE CONTROLLED
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

    // ==================== VOLTAGE-BASED SHOOTER CONTROL ====================
    private static final double TARGET_VOLTAGE = 12.0;           // Start at 12V
    private static final double TARGET_TPS = 1322.222222;        // Target ticks per second
    private static final double TPS_TOLERANCE = 72.0;            // ±72 ticks tolerance (±1250 window around target)
    private static final double MIN_SAFE_TPS = TARGET_TPS - TPS_TOLERANCE;  // 1250 tps
    private static final double MAX_SAFE_TPS = TARGET_TPS + TPS_TOLERANCE;  // 1394 tps

    // Voltage ramping control
    private double currentShooterVoltage = 0.0;                  // Current applied voltage
    private double targetShooterVoltage = TARGET_VOLTAGE;        // Target voltage (starts at 12V)
    private static final double VOLTAGE_RAMP_RATE = 0.3;         // V per loop iteration (smooth ramp)
    private static final double MAX_SHOOTER_VOLTAGE = 12.5;      // Safety limit
    private static final double MIN_SHOOTER_VOLTAGE = 8.0;       // Minimum operating voltage

    // PID-like control for TPS regulation
    private static final double KP_TPS = 0.008;                  // Proportional gain for TPS error
    private static final double VOLTAGE_ADJUST_LIMIT = 0.15;     // Max voltage change per adjustment

    // Shooter state
    private double currentTPS = 0;
    private double calculatedShooterPower = 0.65;
    private double batteryVoltage = 12.0;
    private boolean shooterRamping = false;
    private ElapsedTime rampTimer = new ElapsedTime();

    // ==================== KALMAN FILTER VARIABLES ====================
    // Distance Kalman Filter
    private double filteredDistance = 72.0;
    private double distanceUncertainty = 10.0;
    private static final double DISTANCE_PROCESS_NOISE = 0.5;      // Expected distance change per loop
    private static final double DISTANCE_MEASUREMENT_NOISE = 2.0;  // Sensor noise estimate

    // TPS Kalman Filter
    private double filteredTPS = 1322.0;
    private double tpsUncertainty = 50.0;
    private static final double TPS_PROCESS_NOISE = 5.0;          // Expected TPS change per loop
    private static final double TPS_MEASUREMENT_NOISE = 20.0;     // Encoder noise estimate

    // TX (Turret Horizontal) Kalman Filter
    private double filteredTX = 0.0;
    private double txUncertainty = 1.0;
    private static final double TX_PROCESS_NOISE = 0.1;           // Expected tx change per loop
    private static final double TX_MEASUREMENT_NOISE = 0.5;       // AprilTag noise estimate

    // ==================== DISTANCE CALCULATION ====================
    private double lastValidDistance = 72.0;
    private double lastRawDistance = 72.0;
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double ANGLE_DISTANCE_OFFSET = 0.0;
    private static final double POSE_DISTANCE_OFFSET = 0.0;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 300.0;

    // Debug
    private double rawX = 0, rawY = 0, rawZ = 0;
    private double rawHorizontal = 0;
    private double rawAngleDist = 0;
    private String distanceMethod = "NONE";

    // ==================== POSITIONS ====================
    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.375;
    private static final double SLOT_2_POSITION = 0.78;
    private static final double SHOOT_POSITION_0 = 0.57;
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;
    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    // ==================== TURRET ====================
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double MANUAL_YAW_STEP = 0.01;
    private static final double TX_GAIN = 0.025;
    private static final double PITCH_DEADZONE = 2.0;
    private static final double MAX_PITCH_POWER = 0.9;

    // ==================== INTAKE - OPTIMIZED ====================
    private static final int SHOOT_DELAY = 60;
    private static final int AUTO_ROTATE_DELAY = 250;
    private static final int COLOR_THRESHOLD = 150;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 350;
    private static final double INTAKE_POWER = 0.8;

    // ==================== BALL STORAGE ====================
    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;
    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    // ==================== STATE ====================
    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean isShooting = false;
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true;
    private boolean inCooldown = false;
    private boolean shooterEnabled = false;
    private boolean rollerRunning = false;

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

    // Speed optimizations
    private int telemetryCounter = 0;
    private int colorCheckCounter = 0;
    private static final int TELEMETRY_UPDATE_INTERVAL = 4;
    private static final int COLOR_CHECK_INTERVAL = 2;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Update battery voltage
            updateBatteryVoltage();

            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            // Toggle auto-aim
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastAState = gamepad1.a;

            // Auto-aim or manual
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

            // Auto-detect balls
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            // Scan pattern
            if (gamepad1.y && !lastYState && !isShooting) {
                scanAprilTagPattern();
            }
            lastYState = gamepad1.y;

            // Toggle shooter with VOLTAGE RAMPING
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooterEnabled = !shooterEnabled;
                if (shooterEnabled) {
                    startShooterRamp();
                } else if (!isShooting) {
                    stopShooter();
                }
            }
            lastDpadDownState = gamepad1.dpad_down;

            // Update shooter voltage control
            if (shooterEnabled && !isShooting) {
                updateShooterVoltageControl();
            }

            // Shoot sequence
            if (gamepad1.b && !lastBState && !isShooting) {
                if (ballCount >= MAX_BALLS) {
                    executeShootSequence();
                }
            }
            lastBState = gamepad1.b;

            // Optimized telemetry
            telemetryCounter++;
            if (telemetryCounter >= TELEMETRY_UPDATE_INTERVAL) {
                updateTelemetry();
                telemetryCounter = 0;
            }
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

        // Shooter - voltage controlled
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

    // ==================== VOLTAGE CONTROL FUNCTIONS ====================

    /**
     * Kalman filter for distance measurement
     * Combines noisy Limelight readings with prediction for smoother estimate
     */
    private double filterDistance(double measuredDistance) {
        // PREDICTION STEP
        double predictedDistance = filteredDistance;
        double predictedUncertainty = distanceUncertainty + DISTANCE_PROCESS_NOISE;

        // UPDATE STEP
        double kalmanGain = predictedUncertainty / (predictedUncertainty + DISTANCE_MEASUREMENT_NOISE);

        // Combine prediction and measurement
        filteredDistance = predictedDistance + kalmanGain * (measuredDistance - predictedDistance);
        distanceUncertainty = (1 - kalmanGain) * predictedUncertainty;

        return filteredDistance;
    }

    /**
     * Kalman filter for TPS (shooter velocity)
     * Smooths encoder noise for stable voltage control
     */
    private double filterTPS(double measuredTPS) {
        // PREDICTION STEP
        double predictedTPS = filteredTPS;
        double predictedUncertainty = tpsUncertainty + TPS_PROCESS_NOISE;

        // UPDATE STEP
        double kalmanGain = predictedUncertainty / (predictedUncertainty + TPS_MEASUREMENT_NOISE);

        // Combine prediction and measurement
        filteredTPS = predictedTPS + kalmanGain * (measuredTPS - predictedTPS);
        tpsUncertainty = (1 - kalmanGain) * predictedUncertainty;

        return filteredTPS;
    }

    /**
     * Kalman filter for TX (turret horizontal offset)
     * Smooths AprilTag tracking for stable aim
     */
    private double filterTX(double measuredTX) {
        // PREDICTION STEP
        double predictedTX = filteredTX;
        double predictedUncertainty = txUncertainty + TX_PROCESS_NOISE;

        // UPDATE STEP
        double kalmanGain = predictedUncertainty / (predictedUncertainty + TX_MEASUREMENT_NOISE);

        // Combine prediction and measurement
        filteredTX = predictedTX + kalmanGain * (measuredTX - predictedTX);
        txUncertainty = (1 - kalmanGain) * predictedUncertainty;

        return filteredTX;
    }

    /**
     * Get current battery voltage
     */
    private void updateBatteryVoltage() {
        double voltage = 0.0;
        int count = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage += sensor.getVoltage();
            count++;
        }
        batteryVoltage = voltage / Math.max(count, 1);
    }

    /**
     * Start ramping shooter to 12V
     */
    private void startShooterRamp() {
        targetShooterVoltage = TARGET_VOLTAGE;
        currentShooterVoltage = 0.0;
        shooterRamping = true;
        rampTimer.reset();
    }

    /**
     * Stop shooter
     */
    private void stopShooter() {
        shooter.setPower(0);
        currentShooterVoltage = 0.0;
        targetShooterVoltage = 0.0;
        shooterRamping = false;
    }

    /**
     * Main voltage control loop for shooter
     * 1. Ramp to 12V
     * 2. When TPS reaches 1250-1394 range, adjust voltage based on desired power
     */
    private void updateShooterVoltageControl() {
        // Get current velocity and apply Kalman filter
        double rawTPS = Math.abs(shooter.getVelocity());
        currentTPS = filterTPS(rawTPS);  // ← KALMAN FILTERED

        // PHASE 1: RAMPING TO 12V
        if (shooterRamping) {
            // Smooth ramp to target voltage
            if (currentShooterVoltage < targetShooterVoltage) {
                currentShooterVoltage += VOLTAGE_RAMP_RATE;
                if (currentShooterVoltage > targetShooterVoltage) {
                    currentShooterVoltage = targetShooterVoltage;
                }
            }

            // Apply voltage
            double power = voltageToMotorPower(currentShooterVoltage);
            shooter.setPower(power);

            // Check if we've reached operating TPS range
            if (currentTPS >= MIN_SAFE_TPS && currentTPS <= MAX_SAFE_TPS) {
                shooterRamping = false; // Exit ramping phase
            }

            return;
        }

        // PHASE 2: TPS-BASED VOLTAGE REGULATION
        // Once in range (1250-1394 TPS), adjust voltage based on calculated shooter power

        // Calculate target voltage based on desired shooter power and distance
        double desiredVoltage = calculatedShooterPower * batteryVoltage;

        // Apply PID-like correction if TPS drifts
        double tpsError = TARGET_TPS - currentTPS;
        double voltageCorrection = tpsError * KP_TPS;

        // Limit correction magnitude
        voltageCorrection = Math.max(-VOLTAGE_ADJUST_LIMIT, Math.min(VOLTAGE_ADJUST_LIMIT, voltageCorrection));

        // Combine desired voltage with correction
        targetShooterVoltage = desiredVoltage + voltageCorrection;

        // Safety clamps
        targetShooterVoltage = Math.max(MIN_SHOOTER_VOLTAGE, Math.min(MAX_SHOOTER_VOLTAGE, targetShooterVoltage));

        // Smooth transition to new target
        if (Math.abs(targetShooterVoltage - currentShooterVoltage) > 0.05) {
            double step = (targetShooterVoltage - currentShooterVoltage) * 0.3; // 30% per iteration
            currentShooterVoltage += step;
        } else {
            currentShooterVoltage = targetShooterVoltage;
        }

        // Apply voltage
        double power = voltageToMotorPower(currentShooterVoltage);
        shooter.setPower(power);
    }

    /**
     * Convert desired voltage to motor power (0.0-1.0)
     * Accounts for current battery voltage
     */
    private double voltageToMotorPower(double desiredVoltage) {
        if (batteryVoltage <= 0) return 0;

        double power = desiredVoltage / batteryVoltage;
        return Math.max(0.0, Math.min(1.0, power));
    }

    /**
     * Fast auto-aim with non-blocking pitch control
     */
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

        // Find GOAL tag
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

        // Apply Kalman filter to TX for smooth turret control
        double tx = filterTX(rawTX);  // ← KALMAN FILTERED

        // Calculate distance and apply Kalman filter
        double rawDistance = calculateDistance(target, ty);
        double estimatedDistance = filterDistance(rawDistance);  // ← KALMAN FILTERED
        lastValidDistance = estimatedDistance;

        // Calculate shooter power (used for voltage calculation)
        calculatedShooterPower = calculateShooterPower(estimatedDistance);

        // Calculate yaw
        double targetYawAngle = calculateYawAngle(estimatedDistance);
        yawPosition = angleToServoPosition(targetYawAngle);
        yawPosition += (ty * 0.01); //NP added line01/24
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // Pitch control - non-blocking, using filtered TX
        double pitchPower = 0;
        if (Math.abs(tx) > PITCH_DEADZONE) {
            pitchPower = Math.max(-MAX_PITCH_POWER, Math.min(MAX_PITCH_POWER, -tx * 0.025));  //NP added 01/24
        }
        turretPitch.setPower(pitchPower);
        sleep(200);
        turretPitch.setPower(0);
    }

    private double calculateDistance(LLResultTypes.FiducialResult target, double ty) {
        Pose3D cameraPose = target.getCameraPoseTargetSpace();

        if (cameraPose != null) {
            Position pos = cameraPose.getPosition();

            rawX = pos.x * 39.3701;
            rawY = pos.y * 39.3701;
            rawZ = pos.z * 39.3701;

            rawHorizontal = Math.sqrt(rawZ * rawZ + rawX * rawX);
            double calibratedDistance = rawHorizontal + POSE_DISTANCE_OFFSET;

            distanceMethod = "3D_POSE";
            return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, calibratedDistance));
        }
        else {
            // Jay 01/24

        }
        return calculateDistanceFromAngle(ty);
    }

    private double calculateDistanceFromAngle(double ty) {
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;

        if (Math.abs(totalAngle) > 85) {
            return lastValidDistance;
        }

        double angleRadians = Math.toRadians(totalAngle);
        rawAngleDist = HEIGHT_DIFF / Math.tan(angleRadians);
        double calibratedDistance = rawAngleDist + ANGLE_DISTANCE_OFFSET;

        distanceMethod = "ANGLE";
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
        } else if (gamepad1.right_trigger > 0.1) {
            pitchPower = gamepad1.right_trigger;
        }
        turretPitch.setPower(pitchPower);

        distanceMethod = "MANUAL";
    }

    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS) return;

        if (inCooldown) {
            if (cooldownTimer.milliseconds() > DETECTION_COOLDOWN) {
                inCooldown = false;
            }
            return;
        }

        colorCheckCounter++;
        if (colorCheckCounter < COLOR_CHECK_INTERVAL) {
            return;
        }
        colorCheckCounter = 0;

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

                if (ballCount >= MAX_BALLS) {
                    intakeEnabled = false;
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
        int total = red + green + blue;
        if (total < 100) return null;

        double rNorm = (double)red / total;
        double gNorm = (double)green / total;
        double bNorm = (double)blue / total;

        if (gNorm > 0.45 && gNorm > rNorm && gNorm > bNorm) {
            return "Green";
        }

        if ((rNorm + bNorm) > 0.65 && gNorm < 0.35) {
            return "Purple";
        }

        return null;
    }

    private void rotateSortWheelToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;
        double[] positions = {SLOT_0_POSITION, SLOT_1_POSITION, SLOT_2_POSITION};
        sortWheel.setPosition(positions[currentSlot]);
    }

    private void scanAprilTagPattern() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults() == null) {
            return;
        }

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            int id = (int) fiducial.getFiducialId();
            if (id >= 21 && id <= 23) {
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                return;
            }
        }
    }

    private void setTargetPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21: targetPattern = new String[]{"Green", "Purple", "Purple"}; break;
            case 22: targetPattern = new String[]{"Purple", "Green", "Purple"}; break;
            case 23: targetPattern = new String[]{"Purple", "Purple", "Green"}; break;
            default: targetPattern = null;
        }
    }

    /**
     * MAXIMUM SPEED SHOOT SEQUENCE with voltage control
     */
    private void executeShootSequence() {
        if (isShooting) return;

        isShooting = true;
        roller.setPower(0);
        rollerRunning = false;

        int[] shootOrder = calculateShootOrder();

        // Shooter already at operating voltage/TPS from voltage control
        // Just move to first position
        sortWheel.setPosition(getShootPositionForSlot(shootOrder[0]));
        sleep(300);

        // RAPID FIRE
        for (int i = 0; i < 3; i++) {
            if (i > 0) {
                sortWheel.setPosition(getShootPositionForSlot(shootOrder[i]));
                sleep(120);
            }

            // FIRE
            kicker.setPosition(KICKER_OPEN);
            sleep(SHOOT_DELAY);
            kicker.setPosition(KICKER_CLOSED);
            sleep(SHOOT_DELAY);

            ballSlots[shootOrder[i]] = null;
            ballCount--;
        }

        // Fast cleanup
        shooter.setPower(0);
        shooterEnabled = false;
        currentShooterVoltage = 0.0;
        sortWheel.setPosition(SLOT_0_POSITION);
        currentSlot = 0;
        intakeEnabled = true;
        isShooting = false;
    }

    private int[] calculateShootOrder() {
        int[] order = new int[]{0, 1, 2};

        if (targetPattern == null || !motifDetected) {
            return order;
        }

        String[] available = ballSlots.clone();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (available[j] != null && available[j].equals(targetPattern[i])) {
                    order[i] = j;
                    available[j] = null;
                    break;
                }
            }
        }

        return order;
    }

    private double getShootPositionForSlot(int slotNumber) {
        switch (slotNumber) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
        }
    }

    private void updateTelemetry() {
        // Get encoder position and velocity
        int shooterPosition = shooter.getCurrentPosition();
        double shooterVelocity = shooter.getVelocity();

        telemetry.addData("== ENCODER DATA ==", "");
        telemetry.addData("Shooter Position", "%d ticks", shooterPosition);
        telemetry.addData("Shooter Velocity", "%.1f ticks/sec", shooterVelocity);

        telemetry.addData("== VOLTAGE CONTROL ==", "");
        telemetry.addData("Target Voltage", "%.2f V", targetShooterVoltage);
        telemetry.addData("Current Voltage", "%.2f V", currentShooterVoltage);
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Motor Power", "%.3f", shooter.getPower());
        telemetry.addData("Status", shooterRamping ? "RAMPING" : "REGULATING");

        telemetry.addData("== SHOOTER TPS ==", "");
        telemetry.addData("Current TPS", "%.0f", currentTPS);
        telemetry.addData("Target TPS", "%.0f", TARGET_TPS);
        telemetry.addData("TPS Range", "%.0f - %.0f", MIN_SAFE_TPS, MAX_SAFE_TPS);

        boolean inRange = currentTPS >= MIN_SAFE_TPS && currentTPS <= MAX_SAFE_TPS;
        telemetry.addData("In Range", inRange ? "YES" : "NO");

        telemetry.addData("== KALMAN FILTERS ==", "");
        telemetry.addData("Filtered Distance", "%.1f in", filteredDistance);
        telemetry.addData("Filtered TPS", "%.0f", filteredTPS);
        telemetry.addData("Filtered TX", "%.2f°", filteredTX);

        telemetry.addData("== STATUS ==", "");
        telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
        telemetry.addData("Calc Power", "%.3f", calculatedShooterPower);
        telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);
        telemetry.addData("Intake", rollerRunning ? "ON" : "OFF");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");

        telemetry.update();
    }
}