package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LimelightHelpers;

/**
 * FTC Autonomous with Limelight 3A AprilTag Alignment
 * Uses 2-encoder diagonal odometry (frontRight + backLeft)
 * Hardware: Mecanum drive, REV Control Hub IMU, Limelight 3A
 */
@Autonomous(name = "Auto_LL_Simple", group = "Autonomous")
public class Auto_LL_Simple extends LinearOpMode {

    // ========== HARDWARE ==========
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private Limelight3A limelight;

    // ========== CONSTANTS ==========
    // Encoder counts per revolution (GoBilda 312 RPM motor)
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBilda 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 0.5;    // 2:1 reduction (2 motor revs = 1 wheel rev)
    private static final double WHEEL_DIAMETER_MM = 104.0;     // 104mm mecanum wheels

    // Calculate counts per mm: (encoder counts per wheel revolution) / (wheel circumference)
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * Math.PI);

    // Movement parameters
    private static final double DRIVE_POWER = 0.9;
    private static final double TURN_POWER = 0.3;
    private static final double ALIGN_POWER = 0.25;
    private static final double HEADING_THRESHOLD = 1.0; // degrees

    // ========== ODOMETRY STATE ==========
    private int lastFR = 0;
    private int lastBL = 0;
    private double forwardOdometry = 0.0;  // mm traveled forward
    private double strafeOdometry = 0.0;   // mm traveled right

    @Override
    public void runOpMode() {
        // ========== INITIALIZE HARDWARE ==========
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        // Map motors (adjust names to match your robot configuration)
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Set motor directions (frontLeft & backLeft reversed per spec)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Stop and reset encoders on the two diagonal motors we're using
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to RUN_WITHOUT_ENCODER (we'll manually read encoder values)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Enable zero-power braking
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightHelpers.initLimelight(limelight);

        // Reset odometry tracking
        lastFR = frontRight.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();

        telemetry.addData("Status", "Ready to start");
        telemetry.addData("IMU", "Yaw reset");
        telemetry.addData("Encoders", "FR=%d BL=%d", lastFR, lastBL);
        telemetry.update();

        // ========== WAIT FOR START ==========
        waitForStart();

        if (opModeIsActive()) {
            // ========== AUTONOMOUS SEQUENCE ==========

            // 1. Drive forward 500mm using encoder odometry
            telemetry.addData("Action", "Driving forward 500mm");
            telemetry.update();
            driveForward(500, DRIVE_POWER);

            sleep(500); // Brief pause

            // 2. Strafe right 300mm using diagonal odometry
            telemetry.addData("Action", "Strafing right 300mm");
            telemetry.update();
            strafeRight(300, DRIVE_POWER);

            sleep(500); // Brief pause

            // 3. Use Limelight to align to AprilTag
            telemetry.addData("Action", "Aligning to AprilTag");
            telemetry.update();
            alignToAprilTag();

            sleep(500); // Brief pause

            // 4. Turn to face forward (0 degrees)
            telemetry.addData("Action", "Turning to 0°");
            telemetry.update();
            turnToHeading(0);

            // ========== STOP ALL MOTORS ==========
            stopDrive();
            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    // ========== MOVEMENT METHODS ==========

    /**
     * Drive forward using encoder-based odometry
     * Uses diagonal encoder math: forward = (FR + BL) / 2
     */
    private void driveForward(double targetMM, double power) {
        resetOdometry();

        while (opModeIsActive() && Math.abs(forwardOdometry) < targetMM) {
            updateOdometry();

            // Mecanum forward: all wheels same direction
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Forward Odo", "%.1f / %.1f mm", forwardOdometry, targetMM);
            telemetry.update();
        }
        stopDrive();
    }

    /**
     * Strafe right using encoder-based odometry
     * Uses diagonal encoder math: strafe = (FR - BL) / 2
     */
    private void strafeRight(double targetMM, double power) {
        resetOdometry();

        while (opModeIsActive() && Math.abs(strafeOdometry) < targetMM) {
            updateOdometry();

            // Mecanum strafe right: FL & BR forward, FR & BL reverse
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);

            telemetry.addData("Strafe Odo", "%.1f / %.1f mm", strafeOdometry, targetMM);
            telemetry.update();
        }
        stopDrive();
    }

    /**
     * Turn to a target heading using IMU
     */
    private void turnToHeading(double targetDegrees) {
        while (opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetDegrees - currentHeading;

            // Normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) < HEADING_THRESHOLD) {
                break; // Aligned
            }

            // Proportional turn power
            double turnPower = Math.max(-TURN_POWER, Math.min(TURN_POWER, error * 0.02));

            // Mecanum turn in place: left side forward, right side reverse
            frontLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backLeft.setPower(turnPower);
            backRight.setPower(-turnPower);

            telemetry.addData("Heading", "Current=%.1f Target=%.1f Error=%.1f",
                    currentHeading, targetDegrees, error);
            telemetry.update();
        }
        stopDrive();
    }

    /**
     * Align to AprilTag using Limelight TX (horizontal offset)
     * Assumes LimelightHelpers is imported/available in your project
     */
    private void alignToAprilTag() {
        while (opModeIsActive()) {
            // Get Limelight data
            double tx = LimelightHelpers.getTX(""); // Horizontal offset in degrees
            boolean hasTarget = LimelightHelpers.getTV(""); // Valid target?

            if (!hasTarget) {
                telemetry.addData("Limelight", "No target detected");
                telemetry.update();
                stopDrive();
                sleep(100);
                continue;
            }

            // Check if aligned (within ±1 degree)
            if (Math.abs(tx) < 1.0) {
                telemetry.addData("Limelight", "ALIGNED (TX=%.2f)", tx);
                telemetry.update();
                break;
            }

            // Proportional steering based on TX
            // Positive TX = target is to the right, turn right (clockwise)
            double turnPower = Math.max(-ALIGN_POWER, Math.min(ALIGN_POWER, tx * 0.03));

            // Turn in place
            frontLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backLeft.setPower(turnPower);
            backRight.setPower(-turnPower);

            telemetry.addData("Limelight", "TX=%.2f TurnPower=%.2f", tx, turnPower);
            telemetry.update();
        }
        stopDrive();
    }

    // ========== ODOMETRY HELPERS ==========

    /**
     * Update odometry using diagonal encoders
     * Forward = (FR + BL) / 2
     * Strafe = (FR - BL) / 2
     */
    private void updateOdometry() {
        int currentFR = frontRight.getCurrentPosition();
        int currentBL = backLeft.getCurrentPosition();

        int deltaFR = currentFR - lastFR;
        int deltaBL = currentBL - lastBL;

        // Average the two encoders and convert counts to mm
        double forwardCounts = (deltaFR + deltaBL) / 2.0;
        double strafeCounts = (deltaFR - deltaBL) / 2.0;

        // Convert from counts to mm (divide by COUNTS_PER_MM, not multiply)
        forwardOdometry += forwardCounts / COUNTS_PER_MM;
        strafeOdometry += strafeCounts / COUNTS_PER_MM;

        lastFR = currentFR;
        lastBL = currentBL;
    }

    private void resetOdometry() {
        lastFR = frontRight.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        forwardOdometry = 0.0;
        strafeOdometry = 0.0;
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}