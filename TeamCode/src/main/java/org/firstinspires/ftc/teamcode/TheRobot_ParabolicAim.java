// Version 7 - Parabolic Trajectory Aiming - Aditya Chanda

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

@TeleOp(name="TheRobot_ParabolicAim", group="Competition")
public class TheRobot_ParabolicAim extends LinearOpMode {

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

    // Parabolic trajectory constants
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 45.0;
    private static final double ROBOT_HEIGHT = 12.0;
    private static final double ROBOT_OFFSET_X = 12.2047; // Horizontal offset from shooter to robot center
    private static final double DISTANCE_THRESHOLD = 50.0; // Threshold for using fixed coefficient
    private static final double FIXED_A_COEFFICIENT = -0.02; // For distances > 50

    // Motor and shooter constants
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double COMPRESSION_MM = 10.0;
    private static final double BACKING_ANGLE_CLOSE = 89.0; // degrees, for distance <= 50
    private static final double BACKING_ANGLE_FAR = 20.0;   // degrees, for distance > 50
    private static final double WHEEL_RADIUS_MM = 48.0; // Approximate flywheel radius

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
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
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
    private double lastValidLaunchAngle = 45.0;

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

    // Parabolic trajectory results
    private double calculatedLaunchAngle = 45.0;
    private double calculatedLaunchVelocity = 0.0;
    private double parabolicA = 0.0;
    private double parabolicB = 0.0;
    private double parabolicC = 0.0;

    @Override
    public void runOpMode() {
        initHardware();

        // Initialize Kalman filters
        txFilter = new KalmanFilter(0.0, 1.0, 0.5, 2.0);
        distanceFilter = new KalmanFilter(72.0, 5.0, 1.0, 5.0);

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
            telemetry.addData("Launch Angle", "%.1f°", calculatedLaunchAngle);
            telemetry.addData("Launch Velocity", "%.1f in/s", calculatedLaunchVelocity);
            telemetry.addData("Distance", "%.1f in (%s)", lastValidDistance, distanceMethod);
            telemetry.addData("Parabola", "y = %.4fx² + %.4fx + %.4f", parabolicA, parabolicB, parabolicC);
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

        // Calculate raw distance
        double estimatedDistance = calculateDistance(target, ty);

        // Apply Kalman filtering
        filteredTX = txFilter.update(tx);
        filteredDist = distanceFilter.update(estimatedDistance);
        lastValidDistance = filteredDist;

        // Calculate parabolic trajectory
        ParabolicTrajectory trajectory = calculateParabolicTrajectory(filteredDist);
        calculatedLaunchAngle = trajectory.launchAngle;
        calculatedLaunchVelocity = trajectory.launchVelocity;
        calculatedShooterPower = trajectory.shooterPower;
        parabolicA = trajectory.a;
        parabolicB = trajectory.b;
        parabolicC = trajectory.c;

        // Calculate and set yaw angle based on horizontal distance
        double targetYawAngle = calculateYawAngle(filteredDist);
        yawPosition = angleToServoPosition(targetYawAngle);
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);
        lastValidYawPos = yawPosition;
        lastValidLaunchAngle = calculatedLaunchAngle;

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

        telemetry.addData("tx raw", "%.2f°", tx);
        telemetry.addData("tx filtered", "%.2f°", filteredTX);
        telemetry.addData("ty", "%.2f°", ty);
        telemetry.addData("Distance raw", "%.1f in", estimatedDistance);
        telemetry.addData("Distance filtered", "%.1f in", filteredDist);
        telemetry.addData("Yaw Angle", "%.2f°", targetYawAngle);
        telemetry.addData("Pitch Power", "%.2f (max: %.2f)", pitchPower, currentPitchPower);
    }

    private void handleTrackingLoss() {
        telemetry.addLine("⚠ Lost tracking - maintaining last position");
        turretYaw.setPosition(lastValidYawPos);
        turretPitch.setPower(0);

        // When tracking is lost, increase process noise to allow faster adaptation when target reappears
        txFilter.increaseProcessNoise(1.5);
        distanceFilter.increaseProcessNoise(1.5);

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

    /**
     * Calculate parabolic trajectory based on distance
     * Returns launch angle, velocity, and shooter power needed
     */
    private ParabolicTrajectory calculateParabolicTrajectory(double distanceInches) {
        ParabolicTrajectory traj = new ParabolicTrajectory();

        // Robot position: (distance + offset, robot height)
        double robotX = distanceInches + ROBOT_OFFSET_X;
        double robotY = ROBOT_HEIGHT;

        // Goal position: (0, 45)
        double goalX = GOAL_X;
        double goalY = GOAL_Y;

        // Determine coefficient 'a' based on distance
        double a;
        if (distanceInches > DISTANCE_THRESHOLD) {
            a = FIXED_A_COEFFICIENT;
        } else {
            // Calculate 'a' based on parabola passing through both points
            // Using y = -a*x^2 + b*x + c
            // At goal (0, 45): 45 = c
            // At robot (robotX, robotY): robotY = -a*robotX^2 + b*robotX + 45

            // We need to solve for 'a' and 'b'
            // The derivative at the robot position gives us the launch angle
            // dy/dx = -2*a*x + b

            // For now, use an estimated value based on trajectory
            // This can be refined with actual testing
            a = (goalY - robotY) / (robotX * robotX) * 0.8;
        }

        traj.a = a;
        traj.c = goalY;

        // Solve for b using the robot position
        // robotY = -a*robotX^2 + b*robotX + c
        // b*robotX = robotY + a*robotX^2 - c
        traj.b = (robotY + a * robotX * robotX - traj.c) / robotX;

        // Calculate launch angle from derivative at robot position
        // dy/dx = -2*a*robotX + b
        double slope = -2 * a * robotX + traj.b;
        traj.launchAngle = Math.toDegrees(Math.atan(slope));

        // Ensure launch angle is positive and reasonable
        if (traj.launchAngle < 5) traj.launchAngle = 5;
        if (traj.launchAngle > 85) traj.launchAngle = 85;

        // Calculate required launch velocity using physics
        // Horizontal distance component
        double horizontalDist = robotX;
        double verticalDist = goalY - robotY;

        // Using projectile motion equations:
        // Range = v0^2 * sin(2θ) / g
        // But we need to account for height difference

        double angleRad = Math.toRadians(traj.launchAngle);
        double gravity = 386.22; // inches/s^2 (9.8 m/s^2)

        // Simplified calculation for launch velocity
        // v0 = sqrt(g * distance / sin(2θ))
        // Adjusted for height difference
        double sin2theta = Math.sin(2 * angleRad);
        if (sin2theta < 0.1) sin2theta = 0.1; // Prevent division by very small numbers

        double v0Squared = gravity * horizontalDist / sin2theta;
        traj.launchVelocity = Math.sqrt(Math.abs(v0Squared));

        // Account for height difference - add energy needed to reach higher point
        if (verticalDist > 0) {
            double additionalEnergy = 2 * gravity * verticalDist;
            traj.launchVelocity = Math.sqrt(v0Squared + additionalEnergy);
        }

        // Calculate shooter power based on required velocity
        // Convert velocity to RPM needed
        // v = ω * r, where ω is angular velocity in rad/s
        // RPM = ω * 60 / (2π)

        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS_MM / 25.4; // inches
        double requiredRPM = (traj.launchVelocity * 60) / wheelCircumference;

        // Account for compression and backing angle
        double backingAngle = (distanceInches > DISTANCE_THRESHOLD) ?
                BACKING_ANGLE_FAR : BACKING_ANGLE_CLOSE;
        double compressionFactor = 1.0 + (COMPRESSION_MM / WHEEL_RADIUS_MM) *
                Math.sin(Math.toRadians(backingAngle));

        requiredRPM *= compressionFactor;

        // Convert RPM to motor power (0.0 to 1.0)
        // Assuming linear relationship for now (can be calibrated)
        // Apply 2x power multiplier to account for real-world losses
        traj.shooterPower = (requiredRPM / MOTOR_MAX_RPM) * 2.0;

        // Clamp shooter power to reasonable range
        traj.shooterPower = Math.max(0.50, Math.min(1.0, traj.shooterPower));

        return traj;
    }

    /**
     * Inner class to hold parabolic trajectory calculation results
     */
    private static class ParabolicTrajectory {
        double launchAngle;      // degrees
        double launchVelocity;   // inches/second
        double shooterPower;     // 0.0 to 1.0
        double a, b, c;          // parabola coefficients
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

        telemetry.addData("Complete", "3 balls fired with parabolic trajectory");
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

    /**
     * Kalman Filter implementation for smoothing noisy sensor data
     * Particularly useful for moving camera scenarios
     */
    private static class KalmanFilter {
        private double estimate; // Current state estimate
        private double errorCovariance; // Estimation error covariance
        private double processNoise; // Process noise (how much the system changes)
        private double measurementNoise; // Measurement noise (sensor noise)
        private double initialProcessNoise; // Store initial value for reset

        /**
         * Initialize Kalman filter
         * @param initialEstimate Starting value
         * @param initialErrorCovariance Initial uncertainty
         * @param processNoise How much the true value changes per update (Q)
         * @param measurementNoise Expected sensor noise (R)
         */
        public KalmanFilter(double initialEstimate, double initialErrorCovariance,
                            double processNoise, double measurementNoise) {
            this.estimate = initialEstimate;
            this.errorCovariance = initialErrorCovariance;
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.initialProcessNoise = processNoise;
        }

        /**
         * Update filter with new measurement
         * @param measurement New sensor reading
         * @return Filtered estimate
         */
        public double update(double measurement) {
            // Prediction step
            // Predict the next state (assuming constant)
            double predictedEstimate = estimate;
            double predictedErrorCovariance = errorCovariance + processNoise;

            // Update step
            // Calculate Kalman gain
            double kalmanGain = predictedErrorCovariance /
                    (predictedErrorCovariance + measurementNoise);

            // Update estimate with measurement
            estimate = predictedEstimate + kalmanGain * (measurement - predictedEstimate);

            // Update error covariance
            errorCovariance = (1 - kalmanGain) * predictedErrorCovariance;

            return estimate;
        }

        /**
         * Get current filtered estimate
         */
        public double getEstimate() {
            return estimate;
        }

        /**
         * Get current uncertainty (error covariance)
         */
        public double getUncertainty() {
            return errorCovariance;
        }

        /**
         * Increase process noise temporarily (useful when tracking is lost)
         */
        public void increaseProcessNoise(double factor) {
            processNoise = initialProcessNoise * factor;
        }

        /**
         * Reset process noise to initial value
         */
        public void resetProcessNoise() {
            processNoise = initialProcessNoise;
        }

        /**
         * Reset the filter to initial state
         */
        public void reset(double newEstimate, double newErrorCovariance) {
            estimate = newEstimate;
            errorCovariance = newErrorCovariance;
            processNoise = initialProcessNoise;
        }
    }
}