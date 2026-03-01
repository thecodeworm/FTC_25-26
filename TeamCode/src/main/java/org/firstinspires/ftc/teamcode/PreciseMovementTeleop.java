// Simple TeleOp with Precise Backward Movement
// Configuration: 104mm mecanum wheels, 312 motors, 2:1 gear reduction

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Precise Movement TeleOp", group="Competition")
public class PreciseMovementTeleop extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== ADJUSTABLE DISTANCE VARIABLE =====
    // Change this value to adjust how far the robot moves (in inches)
    private static final double MOVEMENT_DISTANCE_INCHES = 1.0;

    // Robot configuration constants
    private static final double WHEEL_DIAMETER_MM = 104.0;
    private static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    private static final double GEAR_REDUCTION = 2.0; // 2:1 reduction
    private static final int MOTOR_TICKS_PER_REV = 537; // REV HD Hex Motor (312 RPM) encoder ticks

    // Calculated constants
    private static final double TICKS_PER_INCH =
            (MOTOR_TICKS_PER_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Movement parameters
    private static final double MOVEMENT_POWER = 0.3; // Slower for precision

    // Toggle state
    private boolean lastAState = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press A", "Move backward " + MOVEMENT_DISTANCE_INCHES + " inch");
        telemetry.addData("Ticks per inch", "%.1f", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Normal drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            // Press A to move backward precisely
            if (gamepad1.a && !lastAState) {
                moveBackward(MOVEMENT_DISTANCE_INCHES);
            }
            lastAState = gamepad1.a;

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Press A", "Move backward " + MOVEMENT_DISTANCE_INCHES + " inch");
            telemetry.addData("FL Encoder", frontLeft.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRight.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeft.getCurrentPosition());
            telemetry.addData("BR Encoder", backRight.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initHardware() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (matching your existing code)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        resetEncoders();
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
     * Move the robot backward a precise distance
     * @param inches Distance to move backward (positive = backward)
     */
    private void moveBackward(double inches) {
        // Calculate target position in encoder ticks
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        // Reset encoders before movement
        resetEncoders();

        // Set target positions (negative for backward movement)
        frontLeft.setTargetPosition(-targetTicks);
        frontRight.setTargetPosition(-targetTicks);
        backLeft.setTargetPosition(-targetTicks);
        backRight.setTargetPosition(-targetTicks);

        // Switch to RUN_TO_POSITION mode
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        frontLeft.setPower(MOVEMENT_POWER);
        frontRight.setPower(MOVEMENT_POWER);
        backLeft.setPower(MOVEMENT_POWER);
        backRight.setPower(MOVEMENT_POWER);

        // Wait until movement is complete
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("Status", "Moving backward %.2f inches", inches);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("FL Position", "%d / %d",
                    frontLeft.getCurrentPosition(), frontLeft.getTargetPosition());
            telemetry.addData("FR Position", "%d / %d",
                    frontRight.getCurrentPosition(), frontRight.getTargetPosition());
            telemetry.addData("BL Position", "%d / %d",
                    backLeft.getCurrentPosition(), backLeft.getTargetPosition());
            telemetry.addData("BR Position", "%d / %d",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Switch back to normal mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Movement complete!");
        telemetry.update();
        sleep(500);
    }

    /**
     * Reset all motor encoders
     */
    private void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set mode for all drive motors
     */
    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}