package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.List;

@Autonomous(name="UseBLUE", group="Competition")
public class AUTOBLUE extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor roller, turretPitch;
    private Servo kicker, turretYaw;
    private ServoImplEx sortWheel;
    private ColorSensor cs;
    private Limelight3A limelight;
    private VoltageSensor voltageSensor;

    // ============================================
    // INTEGRATED TELEOP AUTO-AIM CONSTANTS
    // ============================================

    // Motor specifications - Gobilda 6000 RPM Yellow Jacket
    private static final double TICKS_PER_REV = 103.6;
    private static final double MAX_RPM = 6000.0;
    private static final double MAX_TICKS_PER_SECOND = (MAX_RPM / 60.0) * TICKS_PER_REV; // 10,360 tps

    // PIDF coefficients - tuned for consistent velocity control
    private static final double SHOOTER_kP = 40.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 20.6;

    // Distance calculation constants
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 300.0;

    // WEIGHTED LOOKUP TABLE - [distance, yaw_position, rpm]
    private static final double[][] SHOOTING_LOOKUP_TABLE = {
            // Distance(in), Yaw Position, RPM
            {0,    0.64, 637},
            {5,    0.64, 695},
            {10,   0.64, 695},
            {20,   0.64, 675},
            {30,   0.64, 700},
            {40,   0.64, 720},
            {50,   0.64, 744},
            {60,   0.59, 760},
            {70,   0.59, 803},
            {80,   0.59, 886.6},
            {90,   0.59, 950},
            {100,  0.59, 984.6},
            {120,  0.59, 970},
            {130,  0.59, 970}
    };
    // Shooter velocity control
    private double targetShooterVelocity = 0;
    private double calculatedShooterVelocity = 7425; // Start at idle velocity
    private double lastValidDistance = 72.0;
    private static final double IDLE_SHOOTER_VELOCITY = 7425;  // ~4300 RPM idle (pre-spin)
    private static final double VELOCITY_TOLERANCE = 0.02;
    private static final int MAX_VELOCITY_CHECK_TIME = 150;
    private static final int VELOCITY_SAMPLE_COUNT = 2;

    // Velocity boost settings
    private static final boolean USE_VELOCITY_BOOST = true;
    private static final double BOOST_MULTIPLIER = 1;
    private static final int BOOST_DURATION_MS = 80;

    private double yawPosition = 0.5;

    // Turret yaw servo tuning
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.65;

    // Auto-aim constants with adaptive ramping
    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 2.5;
    private static final double STUCK_TIME_THRESHOLD = 500;
    private static final double POWER_RAMP_RATE = 0.1;
    private double currentPitchPower = TURRET_PITCH_MIN_SPEED;
    private ElapsedTime stuckTimer = new ElapsedTime();
    private double lastTxError = 0;
    private boolean wasStuck = false;

    // Last valid values for tracking loss
    private double lastValidYawPos = YAW_CENTER;
    private double lastValidShooterVelocity = 0.0;
    private double lastValidTX = 0.0;

    // Kalman Filters
    private KalmanFilter txFilter;
    private KalmanFilter distanceFilter;
    private double filteredTX = 0;
    private double filteredDist = 0;

    // Debug
    private String distanceMethod = "NONE";
    private double rawX = 0, rawY = 0, rawZ = 0;
    private double rawHorizontal = 0;
    private double rawAngleDist = 0;

    // ============================================
    // ORIGINAL AUTONOMOUS CONSTANTS (preserved)
    // ============================================

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
    private static final double SLOT_1_POSITION = 0.4;
    private static final double SLOT_2_POSITION = 0.8;

    // Shooting positions
    private static final double SHOOT_POSITION_0 = 0.609;
    private static final double SHOOT_POSITION_1 = 1.0;
    private static final double SHOOT_POSITION_2 = 0.22;

    private static final double KICKER_OPEN = 0.33333;
    private static final double KICKER_CLOSED = 0.73;

    private static final int SERVO_MOVE_DELAY = 750;
    private static final int SHOOT_DELAY = 375;
    private static final int KICKER_CLOSE_DELAY = 300;
    private static final int BETWEEN_SHOTS_DELAY = 200;

    // Drive calibration
    private static final double INCHES_PER_SECOND_BASE = 26;
    private static final double DEGREES_PER_SECOND_BASE = 95;
    private double driveCalibration = 1.0;
    private double turnCalibration = 1.0;

    // Servo timing
    private static final double SUPER_SERVO_SEC_PER_60DEG = 0.043;
    private static final double SPEED_SERVO_SEC_PER_60DEG = 0.09;
    private static final double SERVO_RANGE_DEGREES = 300.0;
    private static final double SERVO_SAFETY_MARGIN = 1.2;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime test = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        // Initialize Kalman filters
        txFilter = new KalmanFilter(0.0, 1.0, 0.5, 2.0);
        distanceFilter = new KalmanFilter(72.0, 5.0, 1.0, 5.0);

        telemetry.addData("Status", "Initialized with Enhanced Auto-Aim");
        telemetry.addData("Shooter", "Velocity control with lookup table");
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

            driveForward(6, 0.4);
            sleep(50);
            turnLeft(15, 0.4);
            sleep(50);

            // Enable shooter with idle velocity
            shooter.setVelocity(IDLE_SHOOTER_VELOCITY);
            sleep(500);

            telemetry.addData("Status", "Auto-aiming with enhanced system...");
            telemetry.update();

            // Use enhanced auto-aim from TeleOp
            boolean locked = autoAimEnhanced();

            telemetry.addData("Locked", locked ? "Yes" : "No");
            telemetry.addData("Target Velocity", "%.0f tps (%.0f RPM)",
                    calculatedShooterVelocity, (calculatedShooterVelocity/TICKS_PER_REV)*60);
            telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
            telemetry.addData("Yaw Position", "%.3f", yawPosition);
            telemetry.update();

            // Apply calculated shooter velocity
            shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
            sleep(1000);

            test.reset();
            if (motifDetected) {
                shootBallsWithPattern(targetPattern);
            } else {
                shootBallsOptimized();
            }

            stopShooter();
            turnRight(15, 0.4);
            driveForward(12, 0.4);

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

        // Shooter with VELOCITY CONTROL and PIDF tuning
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // CRITICAL for fast accel
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

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

        // Voltage sensor for compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        ballCount = 0;
        currentSlot = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
    }

    // ============================================
    // ENHANCED AUTO-AIM FROM TELEOP
    // ============================================

    /**
     * Enhanced auto-aim with Kalman filtering and lookup table
     * Replaces the old autoAimAndLock() function
     */
    public boolean autoAimEnhanced() {
        ElapsedTime lockTimer = new ElapsedTime();
        lockTimer.reset();

        int stableCount = 0;
        final int STABLE_FRAMES_REQUIRED = 8;
        final double AUTO_AIM_TIMEOUT = 2500;
        final double TARGET_TOLERANCE_TX = 1.5;
        final double TARGET_TOLERANCE_TY = 1.5;

        // Reset turret to center
        yawPosition = YAW_CENTER;
        turretYaw.setPosition(YAW_CENTER);
        turretPitch.setPower(0);
        sleep(150);

        while (opModeIsActive() && lockTimer.milliseconds() < AUTO_AIM_TIMEOUT) {
            LLResult result = limelight.getLatestResult();

            // Check for valid tracking
            if (result == null || !result.isValid()) {
                handleTrackingLoss();
                sleep(10);
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                handleTrackingLoss();
                sleep(10);
                continue;
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

            // Calculate raw distance
            double estimatedDistance = calculateDistanceEnhanced(target, ty);

            // Apply Kalman filtering
            filteredTX = txFilter.update(tx);
            filteredDist = distanceFilter.update(estimatedDistance);
            lastValidDistance = filteredDist;

            // ============ LOOKUP TABLE IMPLEMENTATION ============
            ShootingParameters params = lookupShootingParameters(filteredDist);

            // Set yaw position from lookup table
            yawPosition = params.yawPosition;
            yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
            turretYaw.setPosition(yawPosition);
            lastValidYawPos = yawPosition;

            // Calculate shooter velocity from lookup table RPM
            calculatedShooterVelocity = rpmToTicksPerSecond(params.rpm);
            calculatedShooterVelocity = Math.min(MAX_TICKS_PER_SECOND * 0.95, calculatedShooterVelocity);
            // ====================================================

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

            // Update last valid shooter velocity
            lastValidShooterVelocity = calculatedShooterVelocity;

            // Check stability
            boolean onTarget = (Math.abs(filteredTX) < TARGET_TOLERANCE_TX) &&
                    (Math.abs(ty) < TARGET_TOLERANCE_TY);

            if (onTarget) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            telemetry.addData("TX filtered", "%.2f°", filteredTX);
            telemetry.addData("TY", "%.2f°", ty);
            telemetry.addData("Distance filtered", "%.1f in", filteredDist);
            telemetry.addData("Method", distanceMethod);
            telemetry.addData("Yaw (LUT)", "%.3f", yawPosition);
            telemetry.addData("RPM (LUT)", "%.0f", params.rpm);
            telemetry.addData("Pitch Power", "%.2f", pitchPower);
            telemetry.addData("Stable", "%d/%d", stableCount, STABLE_FRAMES_REQUIRED);
            telemetry.update();

            // Lock on sustained stability
            if (stableCount >= STABLE_FRAMES_REQUIRED) {
                telemetry.addData("Status", "🔒 LOCKED!");
                telemetry.addData("Time", "%.0f ms", lockTimer.milliseconds());
                telemetry.update();
                return true;
            }

            sleep(10); // 10ms loop for ~100Hz update rate
        }

        telemetry.addData("Status", "⏱ TIMEOUT - Using last valid position");
        telemetry.update();
        return false;
    }

    /**
     * Handle tracking loss with Kalman filter noise increase
     */
    private void handleTrackingLoss() {
        distanceMethod = "LOST_TRACKING";
        turretYaw.setPosition(lastValidYawPos);
        turretPitch.setPower(0);

        // Increase process noise when tracking is lost
        txFilter.increaseProcessNoise(1.5);
        distanceFilter.increaseProcessNoise(1.5);
    }

    /**
     * Enhanced distance calculation using 3D pose data
     */
    private double calculateDistanceEnhanced(LLResultTypes.FiducialResult target, double ty) {
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

    /**
     * Fallback distance calculation from angle
     */
    private double calculateDistanceFromAngle(double ty) {
        double totalAngle = CAMERA_MOUNT_ANGLE + ty;
        double angleRadians = Math.toRadians(totalAngle);
        rawAngleDist = HEIGHT_DIFF / Math.tan(angleRadians);
        distanceMethod = String.format("ANGLE(ty:%.2f totalAngle:%.2f)", ty, totalAngle);
        return Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, rawAngleDist));
    }

    /**
     * WEIGHTED LOOKUP TABLE INTERPOLATION
     */
    private ShootingParameters lookupShootingParameters(double distance) {
        // Clamp distance to table bounds
        distance = Math.max(SHOOTING_LOOKUP_TABLE[0][0],
                Math.min(SHOOTING_LOOKUP_TABLE[SHOOTING_LOOKUP_TABLE.length - 1][0], distance));

        // Find bracketing entries
        int lowerIndex = 0;
        int upperIndex = SHOOTING_LOOKUP_TABLE.length - 1;

        for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length - 1; i++) {
            if (distance >= SHOOTING_LOOKUP_TABLE[i][0] && distance <= SHOOTING_LOOKUP_TABLE[i + 1][0]) {
                lowerIndex = i;
                upperIndex = i + 1;
                break;
            }
        }

        // Handle exact match
        if (distance == SHOOTING_LOOKUP_TABLE[lowerIndex][0]) {
            return new ShootingParameters(
                    SHOOTING_LOOKUP_TABLE[lowerIndex][1],
                    SHOOTING_LOOKUP_TABLE[lowerIndex][2]
            );
        }

        if (distance == SHOOTING_LOOKUP_TABLE[upperIndex][0]) {
            return new ShootingParameters(
                    SHOOTING_LOOKUP_TABLE[upperIndex][1],
                    SHOOTING_LOOKUP_TABLE[upperIndex][2]
            );
        }

        // Linear interpolation
        double lowerDist = SHOOTING_LOOKUP_TABLE[lowerIndex][0];
        double upperDist = SHOOTING_LOOKUP_TABLE[upperIndex][0];
        double weight = (distance - lowerDist) / (upperDist - lowerDist);

        double lowerYaw = SHOOTING_LOOKUP_TABLE[lowerIndex][1];
        double upperYaw = SHOOTING_LOOKUP_TABLE[upperIndex][1];
        double interpolatedYaw = lowerYaw + weight * (upperYaw - lowerYaw);

        double lowerRPM = SHOOTING_LOOKUP_TABLE[lowerIndex][2];
        double upperRPM = SHOOTING_LOOKUP_TABLE[upperIndex][2];
        double interpolatedRPM = lowerRPM + weight * (upperRPM - lowerRPM);

        return new ShootingParameters(interpolatedYaw, interpolatedRPM);
    }

    /**
     * Convert RPM to ticks per second
     */
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    /**
     * Compensate for battery voltage drop
     */
    private double compensateForVoltage(double targetVelocity) {
        double currentVoltage = voltageSensor.getVoltage();
        double nominalVoltage = 12.5;

        if (currentVoltage / nominalVoltage >= 0.95) {
            return targetVelocity;
        }

        return targetVelocity * (currentVoltage / nominalVoltage);
    }

    /**
     * Get current velocity in RPM for display
     */
    private double getCurrentRPM() {
        return (shooter.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    /**
     * Simple data class to hold shooting parameters
     */
    private static class ShootingParameters {
        public final double yawPosition;
        public final double rpm;

        public ShootingParameters(double yawPosition, double rpm) {
            this.yawPosition = yawPosition;
            this.rpm = rpm;
        }
    }

    // ============================================
    // SHOOTING SEQUENCE WITH VELOCITY CONTROL
    // ============================================

    public void shootBallsOptimized() {
        shootBallsWithPattern(null);
    }

    public void shootBallsWithPattern(String[] targetPattern) {
        int[] shootOrder = (targetPattern == null) ? new int[]{0, 1, 2} : calculateShootOrder(targetPattern);

        telemetry.addData("Shoot Order", "[%d, %d, %d]", shootOrder[0], shootOrder[1], shootOrder[2]);
        telemetry.update();
        sleep(500);

        // Use velocity boost technique for fast spin-up
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        if (USE_VELOCITY_BOOST) {
            // Apply velocity boost (overshoot)
            double boostedVelocity = Math.min(
                    calculatedShooterVelocity * BOOST_MULTIPLIER,
                    MAX_TICKS_PER_SECOND * 0.95
            );
            shooter.setVelocity(compensateForVoltage(boostedVelocity));
            sleep(BOOST_DURATION_MS);

            // Return to target velocity
            shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
        } else {
            shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
        }

        // Move to first shooting position
        int firstSlot = shootOrder[0];
        double firstShootPosition = getShootPositionForSlot(firstSlot);
        sortWheel.setPosition(firstShootPosition);
        int firstMoveDelay = calculateServoDelay(SLOT_0_POSITION, firstShootPosition, false);
        sleep(Math.max(0, firstMoveDelay));

        // Wait for velocity stabilization
        targetShooterVelocity = calculatedShooterVelocity;
        boolean velocityReady = waitForVelocityStabilization();
        double spinUpTime = shootTimer.milliseconds();

        telemetry.addData("Spin-up Time", "%.0f ms", spinUpTime);
        telemetry.addData("Target Velocity", "%.0f tps (%.0f RPM)",
                targetShooterVelocity, (targetShooterVelocity/TICKS_PER_REV)*60);
        telemetry.addData("Ready", velocityReady ? "YES" : "NO");
        telemetry.update();

        double currentPosition = firstShootPosition;

        // Shoot all three balls
        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            int slotToShoot = shootOrder[shootIndex];
            String ballColor = ballSlots[slotToShoot];

            if (shootIndex > 0) {
                double shootPosition = getShootPositionForSlot(slotToShoot);
                sortWheel.setPosition(shootPosition);

                int moveDelay = calculateServoDelay(currentPosition, shootPosition, false);
                sleep(moveDelay + 100);
                currentPosition = shootPosition;

                velocityReady = waitForVelocityStabilization();
            }

            // PRE-KICK: Fast tap to position the ball
            double preKickPosition = KICKER_CLOSED - 0.1;
            kicker.setPosition(preKickPosition);
            sleep(50);
            kicker.setPosition(KICKER_CLOSED);
            sleep(50);

            // FULL KICK: Open completely to shoot
            kicker.setPosition(KICKER_OPEN);
            sleep(calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100);
            sleep(SHOOT_DELAY);

            // RESET: Return to closed position
            kicker.setPosition(KICKER_CLOSED);
            sleep(calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true) + 130);

            telemetry.addData("Ball " + (shootIndex + 1), "FIRED");
            telemetry.addData("Velocity", "%.0f tps (%.0f RPM)",
                    shooter.getVelocity(), getCurrentRPM());
            telemetry.update();
        }

        telemetry.addData("Complete", "3 balls fired with lookup table");
        telemetry.update();
    }

    /**
     * Wait for shooter velocity to stabilize
     */
    private boolean waitForVelocityStabilization() {
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();
        int consecutiveGoodReadings = 0;

        while (velocityTimer.milliseconds() < MAX_VELOCITY_CHECK_TIME) {
            double currentVelocity = shooter.getVelocity();
            double velocityError = Math.abs(currentVelocity - targetShooterVelocity) / targetShooterVelocity;

            if (velocityError <= VELOCITY_TOLERANCE) {
                consecutiveGoodReadings++;
                if (consecutiveGoodReadings >= 2) {
                    return true;
                }
            } else {
                consecutiveGoodReadings = 0;
            }

            sleep(10);
        }

        return consecutiveGoodReadings > 0;
    }

    private int calculateServoDelay(double startPosition, double endPosition, boolean isSpeedMode) {
        double positionDelta = Math.abs(endPosition - startPosition);
        double degreesDelta = positionDelta * SERVO_RANGE_DEGREES;
        double secPer60Deg = isSpeedMode ? SPEED_SERVO_SEC_PER_60DEG : SUPER_SERVO_SEC_PER_60DEG;
        double timeSeconds = (degreesDelta / 60.0) * secPer60Deg;
        int delayMs = (int) (timeSeconds * 1000 * SERVO_SAFETY_MARGIN);
        return Math.max(50, delayMs);
    }

    // ============================================
    // ORIGINAL AUTONOMOUS FUNCTIONS (preserved)
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
            case 21: targetPattern = new String[]{"Green", "Purple", "Purple"}; break;
            case 22: targetPattern = new String[]{"Purple", "Green", "Purple"}; break;
            case 23: targetPattern = new String[]{"Purple", "Purple", "Green"}; break;
            default: targetPattern = null;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s, %s, %s]", targetPattern[0], targetPattern[1], targetPattern[2]);
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

    public void stopShooter() {
        shooter.setVelocity(0);
    }

    public void stopDrive() {
        mecanum(0, 0, 0);
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

    // ============================================
    // KALMAN FILTER CLASS
    // ============================================

    /**
     * Kalman Filter implementation for smoothing noisy sensor data
     */
    private static class KalmanFilter {
        private double estimate;
        private double errorCovariance;
        private double processNoise;
        private double measurementNoise;
        private double initialProcessNoise;

        public KalmanFilter(double initialEstimate, double initialErrorCovariance,
                            double processNoise, double measurementNoise) {
            this.estimate = initialEstimate;
            this.errorCovariance = initialErrorCovariance;
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.initialProcessNoise = processNoise;
        }

        public double update(double measurement) {
            double predictedEstimate = estimate;
            double predictedErrorCovariance = errorCovariance + processNoise;

            double kalmanGain = predictedErrorCovariance /
                    (predictedErrorCovariance + measurementNoise);

            estimate = predictedEstimate + kalmanGain * (measurement - predictedEstimate);
            errorCovariance = (1 - kalmanGain) * predictedErrorCovariance;

            return estimate;
        }

        public void increaseProcessNoise(double factor) {
            processNoise = initialProcessNoise * factor;
        }

        public void resetProcessNoise() {
            processNoise = initialProcessNoise;
        }
    }
}