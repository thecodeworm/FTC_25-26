// Version 8 - Aditya Chanda - WEIGHTED LOOKUP TABLE IMPLEMENTATION + Async shooting
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

@TeleOp(name="Async_Lookup", group="Competition")
public class TheRobot_Async_Lookup extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system
    private DcMotorEx shooter;
    private DcMotor roller;
    private Servo kicker;
    private VoltageSensor voltageSensor;

    // Sort wheel / Spindexer
    private ServoImplEx sortWheel;
    private ColorSensor cs;

    // Turret control
    private Servo turretYaw;
    private DcMotor turretPitch;

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // Motor specifications - Gobilda 6000 RPM Yellow Jacket
    // Verified: 1500 RPM = 2590 tps → 103.6 ticks/rev
    private static final double TICKS_PER_REV = 103.6;
    private static final double MAX_RPM = 6000.0;
    private static final double MAX_TICKS_PER_SECOND = (MAX_RPM / 60.0) * TICKS_PER_REV; // 10,360 tps

    // PIDF coefficients - tuned for consistent velocity control
    private static final double SHOOTER_kP = 40.0;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 20.6;

    // Shooter velocity control
    private double targetShooterVelocity = 0;
    private double calculatedShooterVelocity = 120000; // Start at idle velocity
    private double lastValidDistance = 72.0;
    private static final double IDLE_SHOOTER_VELOCITY = 7425; // ~4300 RPM idle (pre-spin)
    private static final double VELOCITY_TOLERANCE = 0.02;
    private static final int MAX_VELOCITY_CHECK_TIME = 150; // Reduced from 300ms
    private static final int VELOCITY_SAMPLE_COUNT = 2; // Reduced from 3 for faster checks

    // Velocity boost settings
    private static final boolean USE_VELOCITY_BOOST = true;
    private static final double BOOST_MULTIPLIER = 0.9;
    private static final int BOOST_DURATION_MS = 80;

    // Distance calculation constants
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 300.0;

    // Storage positions
    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.4;
    private static final double SLOT_2_POSITION = 0.8;

    // Shooting positions
    private static final double SHOOT_POSITION_0 = 0.609;
    private static final double SHOOT_POSITION_1 = 1.0;
    private static final double SHOOT_POSITION_2 = 0.22;

    // Kicker positions
    private static final double KICKER_OPEN = 0.33333;
    private static final double KICKER_CLOSED = 0.73;

    // Turret yaw servo tuning - adjusted for lookup table range
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.0;
    private static final double YAW_MAX = 0.7; // Increased to accommodate lookup table values
    private static final double MANUAL_YAW_STEP = 0.01;

    // WEIGHTED LOOKUP TABLE - [distance, yaw_position, rpm]
    private static final double[][] SHOOTING_LOOKUP_TABLE = {
            // Distance(in), Yaw Position, RPM
            // {0, 0.64, 625},
            // {5, 0.64, 625},
            {10, 0.64, 625},
            {20, 0.64, 625},
            {30, 0.64, 625},
            {40, 0.64, 650},
            {50, 0.64, 675},
            {60, 0.59, 710},
            {70, 0.59, 720},
            {80, 0.59, 763},
            {90, 0.59, 780},
            {100, 0.59, 810},
            {110, 0.59, 850},
            {120, 0.59, 860},
            {130, 0.59, 860},
            {140, 0.59, 845},
            {150, 0.59, 845},
            {160, 0.59, 845}
    };

    // Auto-aim constants
    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 2.5;

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
    private static final int COLOR_THRESHOLD = 130;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 400;
    private static final double INTAKE_POWER = 0.6;

    // Servo specs
    private static final double SUPER_SERVO_SEC_PER_60DEG = 0.043;
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
    private double lastValidShooterVelocity = 0.0;
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

    // Kalman Filters
    private KalmanFilter txFilter;
    private KalmanFilter distanceFilter;
    private double filteredTX = 0;
    private double filteredDist = 0;

    // ========== ASYNC SHOOTING STATE MACHINE ==========
    private enum ShootState {
        IDLE,
        INITIALIZING,
        SPINNING_UP,
        WAITING_FOR_VELOCITY,
        SHOOTING_BALL_1,
        RELOADING_BALL_2,
        SHOOTING_BALL_2,
        RELOADING_BALL_3,
        SHOOTING_BALL_3,
        CLEANUP
    }

    private ShootState shootState = ShootState.IDLE;
    private ElapsedTime shootStateTimer = new ElapsedTime();
    private int[] shootOrder = new int[3];
    private int currentShootIndex = 0;
    private double currentSortWheelPosition = SLOT_0_POSITION;
    private boolean shootSequenceRequested = false;

    @Override
    public void runOpMode() {
        initHardware();

        // Initialize Kalman filters
        txFilter = new KalmanFilter(0.0, 1.0, 0.5, 2.0);
        distanceFilter = new KalmanFilter(72.0, 5.0, 1.0, 5.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drive controls - ALWAYS ACTIVE
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
                    shooter.setVelocity(IDLE_SHOOTER_VELOCITY);
                    telemetry.addData("Shooter", "ENABLED at idle %.0f RPM", getCurrentRPM());
                } else if (!isShooting) {
                    shooter.setVelocity(0);
                    targetShooterVelocity = 0;
                    telemetry.addData("Shooter", "DISABLED");
                }
                telemetry.update();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // Request shoot sequence with B button
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState && shootState == ShootState.IDLE) {
                if (ballCount < MAX_BALLS) {
                    telemetry.addData("Error", "Not enough balls! Have " + ballCount + ", need " + MAX_BALLS);
                    telemetry.update();
                    sleep(750);
                } else {
                    shootSequenceRequested = true;
                }
            }
            lastBState = currentBState;

            // ========== RUN ASYNC SHOOT STATE MACHINE ==========
            updateShootStateMachine();

            // Display telemetry
            if (shooterEnabled) {
                double currentVel = shooter.getVelocity();
                double currentRPM = getCurrentRPM();
                telemetry.addData("Current Velocity", "%.0f tps (%.0f RPM)", currentVel, currentRPM);
                if (targetShooterVelocity > 0) {
                    double error = Math.abs(currentVel - targetShooterVelocity) / targetShooterVelocity * 100;
                    telemetry.addData("Velocity Error", "%.1f%%", error);
                }
            }

            telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("Roller", rollerRunning ? "ON" : "OFF");
            telemetry.addData("Target Velocity", "%.0f tps", calculatedShooterVelocity);
            telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
            telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
            telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);

            if (shootState != ShootState.IDLE) {
                telemetry.addData("Shoot State", shootState.toString());
                telemetry.addData("Ball Progress", "%d/3", currentShootIndex + 1);
            }

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

        // Shooter system with TUNED PIDF CONTROL (P=40, I=0, D=0, F=20.6)
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // CRITICAL for fast accel

        // Set PIDF coefficients for velocity control
        shooter.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

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

        // Voltage sensor for compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

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

    /**
     * ASYNCHRONOUS SHOOT STATE MACHINE
     * Runs incrementally each loop iteration, allowing drive controls to remain active
     */
    private void updateShootStateMachine() {
        switch (shootState) {
            case IDLE:
                if (shootSequenceRequested) {
                    shootSequenceRequested = false;
                    initializeShootSequence();
                }
                break;

            case INITIALIZING:
                // Prepare for shooting - calculate shoot order
                calculateShootOrder();

                // Disable intake
                roller.setPower(0);
                rollerRunning = false;
                intakeEnabled = false;
                inCooldown = true;
                isShooting = true;

                // Move to first ball position
                int firstSlot = shootOrder[0];
                double firstShootPosition = getShootPositionForSlot(firstSlot);
                sortWheel.setPosition(firstShootPosition);
                currentSortWheelPosition = firstShootPosition;

                shootState = ShootState.SPINNING_UP;
                shootStateTimer.reset();
                break;

            case SPINNING_UP:
                // Wait for sort wheel to reach position
                int firstMoveDelay = calculateServoDelay(SLOT_0_POSITION, currentSortWheelPosition, false);

                if (shootStateTimer.milliseconds() >= firstMoveDelay) {
                    // Start shooter spin-up
                    if (USE_VELOCITY_BOOST) {
                        double boostedVelocity = Math.min(
                                calculatedShooterVelocity * BOOST_MULTIPLIER,
                                MAX_TICKS_PER_SECOND * 0.95
                        );
                        shooter.setVelocity(compensateForVoltage(boostedVelocity));
                    } else {
                        shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
                    }

                    targetShooterVelocity = calculatedShooterVelocity;
                    shootState = ShootState.WAITING_FOR_VELOCITY;
                    shootStateTimer.reset();
                }
                break;

            case WAITING_FOR_VELOCITY:
                // Check if boost duration elapsed
                if (USE_VELOCITY_BOOST && shootStateTimer.milliseconds() >= BOOST_DURATION_MS) {
                    shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
                }

                // Check velocity stabilization (non-blocking)
                if (isVelocityStable()) {
                    currentShootIndex = 0;
                    shootState = ShootState.SHOOTING_BALL_1;
                    shootStateTimer.reset();
                }

                // Timeout after reasonable wait
                if (shootStateTimer.milliseconds() > MAX_VELOCITY_CHECK_TIME + 500) {
                    currentShootIndex = 0;
                    shootState = ShootState.SHOOTING_BALL_1;
                    shootStateTimer.reset();
                }
                break;

            case SHOOTING_BALL_1:
                shootBallAsync(0, ShootState.RELOADING_BALL_2);
                break;

            case RELOADING_BALL_2:
                reloadNextBallAsync(1, ShootState.SHOOTING_BALL_2);
                break;

            case SHOOTING_BALL_2:
                shootBallAsync(1, ShootState.RELOADING_BALL_3);
                break;

            case RELOADING_BALL_3:
                reloadNextBallAsync(2, ShootState.SHOOTING_BALL_3);
                break;

            case SHOOTING_BALL_3:
                shootBallAsync(2, ShootState.CLEANUP);
                break;

            case CLEANUP:
                // Reset everything
                shooter.setVelocity(0);
                shooterEnabled = false;
                targetShooterVelocity = 0;
                sortWheel.setPosition(SLOT_0_POSITION);
                currentSlot = 0;
                intakeEnabled = true;
                inCooldown = false;
                ballDetectedInSlot = false;
                isShooting = false;

                shootState = ShootState.IDLE;
                break;
        }
    }

    private void initializeShootSequence() {
        shootState = ShootState.INITIALIZING;
        currentShootIndex = 0;
        shootStateTimer.reset();
    }

    private void calculateShootOrder() {
        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

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
    }

    /**
     * Non-blocking ball shooting - uses state timer
     */
    private void shootBallAsync(int ballIndex, ShootState nextState) {
        int slotToShoot = shootOrder[ballIndex];

        if (shootStateTimer.milliseconds() < calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100) {
            // Open kicker (only on first call)
            if (shootStateTimer.milliseconds() < 10) {
                kicker.setPosition(KICKER_OPEN);
            }
            return; // Still opening
        }

        if (shootStateTimer.milliseconds() < calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100 + SHOOT_DELAY) {
            return; // Waiting for ball to exit
        }

        if (shootStateTimer.milliseconds() < calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100 + SHOOT_DELAY +
                calculateServoDelay(KICKER_OPEN, KICKER_CLOSED, true) + 130) {
            // Close kicker (only on first call of this phase)
            if (shootStateTimer.milliseconds() < calculateServoDelay(KICKER_CLOSED, KICKER_OPEN, true) + 100 + SHOOT_DELAY + 10) {
                kicker.setPosition(KICKER_CLOSED);
            }
            return; // Still closing
        }

        // Ball shot complete
        ballSlots[slotToShoot] = null;
        ballCount--;

        shootState = nextState;
        shootStateTimer.reset();
    }

    /**
     * Non-blocking reload - moves sort wheel to next ball
     */
    private void reloadNextBallAsync(int nextBallIndex, ShootState nextState) {
        int nextSlot = shootOrder[nextBallIndex];
        double nextShootPosition = getShootPositionForSlot(nextSlot);

        int moveDelay = calculateServoDelay(currentSortWheelPosition, nextShootPosition, false);

        if (shootStateTimer.milliseconds() < 10) {
            // Start move (only on first call)
            sortWheel.setPosition(nextShootPosition);
        }

        if (shootStateTimer.milliseconds() >= moveDelay + 100) {
            // Move complete, check velocity
            currentSortWheelPosition = nextShootPosition;

            if (isVelocityStable()) {
                shootState = nextState;
                shootStateTimer.reset();
            } else if (shootStateTimer.milliseconds() > moveDelay + 300) {
                // Timeout, proceed anyway
                shootState = nextState;
                shootStateTimer.reset();
            }
        }
    }

    /**
     * Non-blocking velocity check
     */
    private boolean isVelocityStable() {
        double currentVelocity = shooter.getVelocity();
        double velocityError = Math.abs(currentVelocity - targetShooterVelocity) / targetShooterVelocity;
        return velocityError <= VELOCITY_TOLERANCE;
    }

    /**
     * WEIGHTED LOOKUP TABLE INTERPOLATION
     * Finds the closest two table entries and performs linear interpolation
     * between them based on distance.
     *
     * @param distance Distance to target in inches
     * @return ShootingParameters object containing yaw position and RPM
     */
    private ShootingParameters lookupShootingParameters(double distance) {
        // Clamp distance to table bounds
        distance = Math.max(SHOOTING_LOOKUP_TABLE[0][0],
                Math.min(SHOOTING_LOOKUP_TABLE[SHOOTING_LOOKUP_TABLE.length - 1][0], distance));

        // Find bracketing entries
        int lowerIndex = 0;
        int upperIndex = SHOOTING_LOOKUP_TABLE.length - 1;

        // Binary search for efficiency
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
                    SHOOTING_LOOKUP_TABLE[lowerIndex][1], // yaw position
                    SHOOTING_LOOKUP_TABLE[lowerIndex][2] // rpm
            );
        }

        if (distance == SHOOTING_LOOKUP_TABLE[upperIndex][0]) {
            return new ShootingParameters(
                    SHOOTING_LOOKUP_TABLE[upperIndex][1], // yaw position
                    SHOOTING_LOOKUP_TABLE[upperIndex][2] // rpm
            );
        }

        // Linear interpolation
        double lowerDist = SHOOTING_LOOKUP_TABLE[lowerIndex][0];
        double upperDist = SHOOTING_LOOKUP_TABLE[upperIndex][0];
        double weight = (distance - lowerDist) / (upperDist - lowerDist);

        // Interpolate yaw position
        double lowerYaw = SHOOTING_LOOKUP_TABLE[lowerIndex][1];
        double upperYaw = SHOOTING_LOOKUP_TABLE[upperIndex][1];
        double interpolatedYaw = lowerYaw + weight * (upperYaw - lowerYaw);

        // Interpolate RPM
        double lowerRPM = SHOOTING_LOOKUP_TABLE[lowerIndex][2];
        double upperRPM = SHOOTING_LOOKUP_TABLE[upperIndex][2];
        double interpolatedRPM = lowerRPM + weight * (upperRPM - lowerRPM);

        return new ShootingParameters(interpolatedYaw, interpolatedRPM);
    }

    /**
     * Convert RPM to ticks per second for motor velocity control
     */
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
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

        // Calculate raw distance
        double estimatedDistance = calculateDistance(target, ty);

        // Apply Kalman filtering
        filteredTX = txFilter.update(tx);
        filteredDist = distanceFilter.update(estimatedDistance);
        lastValidDistance = filteredDist;

        // ============ LOOKUP TABLE IMPLEMENTATION ============
        // Get shooting parameters from weighted lookup table
        ShootingParameters params = lookupShootingParameters(filteredDist);

        // Set yaw position from lookup table
        yawPosition = params.yawPosition;
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);
        lastValidYawPos = yawPosition;

        // Calculate shooter velocity from lookup table RPM
        calculatedShooterVelocity = rpmToTicksPerSecond(params.rpm);

        // Clamp to safe motor limits (max 95% of 6000 RPM)
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

        // Update shooter with velocity control
        lastValidShooterVelocity = calculatedShooterVelocity;
        if (shooterEnabled && !isShooting) {
            shooter.setVelocity(compensateForVoltage(calculatedShooterVelocity));
        }

        telemetry.addData("tx raw", "%.2f°", tx);
        telemetry.addData("tx filtered", "%.2f°", filteredTX);
        telemetry.addData("ty", "%.2f°", ty);
        telemetry.addData("Distance raw", "%.1f in", estimatedDistance);
        telemetry.addData("Distance filtered", "%.1f in", filteredDist);
        telemetry.addData("Yaw Position (LUT)", "%.3f", yawPosition);
        telemetry.addData("Target RPM (LUT)", "%.0f", params.rpm);
        telemetry.addData("Pitch Power", "%.2f (max: %.2f)", pitchPower, currentPitchPower);
    }

    private void handleTrackingLoss() {
        telemetry.addLine("⚠ Lost tracking - maintaining last position");
        turretYaw.setPosition(lastValidYawPos);
        turretPitch.setPower(0);

        // When tracking is lost, increase process noise
        txFilter.increaseProcessNoise(1.5);
        distanceFilter.increaseProcessNoise(1.5);

        if (shooterEnabled && !isShooting) {
            shooter.setVelocity(compensateForVoltage(lastValidShooterVelocity));
        } else if (!isShooting) {
            shooter.setVelocity(0);
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
            shooter.setVelocity(IDLE_SHOOTER_VELOCITY);
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
        double secPer60Deg = isSpeedMode ? SPEED_SERVO_SEC_PER_60DEG : SUPER_SERVO_SEC_PER_60DEG;
        double timeSeconds = (degreesDelta / 60.0) * secPer60Deg;
        int delayMs = (int) (timeSeconds * 1000 * SERVO_SAFETY_MARGIN);
        return Math.max(50, delayMs);
    }

    private double getShootPositionForSlot(int slotNumber) {
        switch (slotNumber) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
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
                sleep(10); // Reduced from 20ms
            }
        }
        return sum / VELOCITY_SAMPLE_COUNT;
    }

    /**
     * OPTIMIZED: Wait for shooter velocity to stabilize within tolerance
     * Now much faster with reduced check time and fewer required readings
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
                if (consecutiveGoodReadings >= 2) { // Only need 2 consecutive good readings
                    return true;
                }
            } else {
                consecutiveGoodReadings = 0;
            }

            sleep(10); // Check every 10ms instead of 50ms
        }

        // After timeout, check if we're close enough
        return consecutiveGoodReadings > 0;
    }

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

        public double getEstimate() {
            return estimate;
        }

        public double getUncertainty() {
            return errorCovariance;
        }

        public void increaseProcessNoise(double factor) {
            processNoise = initialProcessNoise * factor;
        }

        public void resetProcessNoise() {
            processNoise = initialProcessNoise;
        }

        public void reset(double newEstimate, double newErrorCovariance) {
            estimate = newEstimate;
            errorCovariance = newErrorCovariance;
            processNoise = initialProcessNoise;
        }
    }
}