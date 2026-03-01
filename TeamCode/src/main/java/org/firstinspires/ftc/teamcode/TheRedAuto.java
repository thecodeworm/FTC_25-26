package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name="TheRedAuto", group="Competition")
public class TheRedAuto extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor roller, turretPitch;
    private Servo kicker, turretYaw;
    private ServoImplEx sortWheel;
    private ColorSensor cs;

    // Manual shooting settings - ADJUST THESE VALUES
    private double manualShooterPower = 0.65;
    private double manualYawPosition = 0.5;  // 0.25 to 0.5828
    private int manualPitchPosition = 0;     // In ticks (negative = down, positive = up)

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

    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.4;
    private static final double SLOT_2_POSITION = 0.8;

    // Shooting positions
    private static final double SHOOT_POSITION_0 = 0.609;
    private static final double SHOOT_POSITION_1 = 1.0;
    private static final double SHOOT_POSITION_2 = 0.22;

    private static final double KICKER_OPEN = 0.33333;
    private static final double KICKER_CLOSED = 0.73;
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;

    private static final int PITCH_TICKS_PER_DEGREE = 20;
    private static final double PITCH_POWER = 0.6;

    private static final int SERVO_MOVE_DELAY = 750;
    private static final int SHOOT_DELAY = 375;
    private static final int KICKER_CLOSE_DELAY = 300;
    private static final int BETWEEN_SHOTS_DELAY = 200;
    private static final int MOTOR_SPINUP_TIME = 1000;

    // Drive calibration
    private static final double INCHES_PER_SECOND_BASE = 26;
    private static final double DEGREES_PER_SECOND_BASE = 95;
    private double driveCalibration = 1.0;
    private double turnCalibration = 1.0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime test = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter Power", "%.2f", manualShooterPower);
        telemetry.addData("Yaw Position", "%.3f", manualYawPosition);
        telemetry.addData("Pitch Position", "%d ticks", manualPitchPosition);
        telemetry.update();

        waitForStart();
        runtime.reset();
        test.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Detecting balls");
            telemetry.update();

            autoDetectBalls(3000);

            telemetry.addData("Balls", "%d/%d", ballCount, MAX_BALLS);
            telemetry.update();

            // Your autonomous path
            driveForward(6, 0.4);
            sleep(50);
            turnRight(15, 0.4);
            sleep(50);

            // Manual aim and shoot
            setYawPosition(0.6);
            setShooterPower(0.67);
            setManualAimAndShoot();

            turnLeft(15, 0.4);
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
        turretPitch.setTargetPosition(0);
        turretPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretPitch.setPower(PITCH_POWER);

        ballCount = 0;
        currentSlot = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
    }

    // ============================================
    // MANUAL SHOOTING METHODS
    // ============================================

    /**
     * Set the shooter power manually (0.0 to 1.0)
     */
    public void setShooterPower(double power) {
        manualShooterPower = Math.max(0.0, Math.min(1.0, power));
        telemetry.addData("Shooter Power Set", "%.2f", manualShooterPower);
        telemetry.update();
    }

    /**
     * Set the turret yaw position manually (0.25 to 0.5828)
     * 0.25 = far left, 0.5 = center, 0.5828 = far right
     */
    public void setYawPosition(double position) {
        manualYawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, position));
        telemetry.addData("Yaw Position Set", "%.3f", manualYawPosition);
        telemetry.update();
    }

    /**
     * Set the turret pitch position manually (in ticks)
     * Negative = aim down, Positive = aim up
     * Rough guide: 20 ticks per degree (calibrate this!)
     */
    public void setPitchPosition(int ticks) {
        manualPitchPosition = ticks;
        telemetry.addData("Pitch Position Set", "%d ticks", manualPitchPosition);
        telemetry.update();
    }

    /**
     * Set the turret pitch by degrees (more intuitive)
     * Negative = aim down, Positive = aim up
     */
    public void setPitchDegrees(double degrees) {
        manualPitchPosition = (int)(degrees * PITCH_TICKS_PER_DEGREE);
        telemetry.addData("Pitch Degrees Set", "%.1f° (%d ticks)", degrees, manualPitchPosition);
        telemetry.update();
    }

    /**
     * Apply the manual aim settings and shoot all balls
     */
    private void setManualAimAndShoot() {
        telemetry.addData("Status", "Setting manual aim...");
        telemetry.addData("Power", "%.2f", manualShooterPower);
        telemetry.addData("Yaw", "%.3f", manualYawPosition);
        telemetry.addData("Pitch", "%d ticks", manualPitchPosition);
        telemetry.update();

        // Apply the manual settings
        turretYaw.setPosition(manualYawPosition);
        turretPitch.setTargetPosition(manualPitchPosition);
        sleep(500);  // Wait for servos to move

        // Spin up shooter
        shooter.setPower(manualShooterPower);
        sleep(MOTOR_SPINUP_TIME);

        // Shoot all balls
        shootBallsOptimized();

        // Stop shooter
        shooter.setPower(0);
    }

    public void shootBallsOptimized() {
        int[] shootOrder = new int[]{0, 1, 2};

        telemetry.addData("Shoot Order", "[%d, %d, %d]", shootOrder[0], shootOrder[1], shootOrder[2]);
        telemetry.update();
        sleep(500);

        // Ball 1
        telemetry.addData("Shooting", "Ball 1 from slot %d", shootOrder[0]);
        telemetry.update();
        int slot1 = shootOrder[0];
        double ball1Power = manualShooterPower * BALL_1_POWER_MULTIPLIER;
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
        double ball2Power = manualShooterPower * BALL_2_POWER_MULTIPLIER;
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
        int slot3 = shootOrder[2];
        double ball3Power = manualShooterPower * BALL_3_POWER_MULTIPLIER;
        shooter.setPower(ball3Power);
        sortWheel.setPosition(getShootPositionForSlot(slot3));
        sleep(SERVO_MOVE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(SHOOT_DELAY + 100);
        kicker.setPosition(KICKER_CLOSED);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_OPEN);
        sleep(KICKER_CLOSE_DELAY);
        kicker.setPosition(KICKER_CLOSED);

        telemetry.addData("Complete", "All 3 balls fired");
        telemetry.update();
    }

    // ============================================
    // DRIVE METHODS
    // ============================================

    public void setCalibratedDistance(double targetInches, double actualInches) {
        driveCalibration = targetInches / actualInches;
        telemetry.addData("Drive Cal", "%.3f (%.1f→%.1f)",
                driveCalibration, targetInches, actualInches);
        telemetry.update();
    }

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
    // BALL DETECTION METHODS
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
            telemetry.addData("Detecting", "%d/%d balls", ballCount, MAX_BALLS);
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

    // ============================================
    // UTILITY METHODS
    // ============================================

    private double getShootPositionForSlot(int slot) {
        switch (slot) {
            case 0: return SHOOT_POSITION_0;
            case 1: return SHOOT_POSITION_1;
            case 2: return SHOOT_POSITION_2;
            default: return SHOOT_POSITION_0;
        }
    }

    public void centerTurret() {
        turretYaw.setPosition(YAW_CENTER);
        turretPitch.setTargetPosition(0);
    }

    public void stopDrive() {
        mecanum(0, 0, 0);
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
}