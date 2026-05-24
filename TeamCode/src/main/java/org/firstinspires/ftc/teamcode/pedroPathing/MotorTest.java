package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Direction Test", group = "Test")
public class MotorTest extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Track which motors are reversed
    private boolean frontLeftReversed  = false;
    private boolean frontRightReversed = false;
    private boolean backLeftReversed   = false;
    private boolean backRightReversed  = false;

    // Track previous button states to detect rising edge (press, not hold)
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {

        // Initialize motors from hardware map
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Set all motors to FORWARD to start
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Motor Direction Test Ready");
        telemetry.addLine("Press A=FL  B=FR  X=BL  Y=BR to toggle direction");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Toggle directions on button press (rising edge only) ---

            // A  →  Front Left
            if (gamepad1.a && !prevA) {
                frontLeftReversed = !frontLeftReversed;
                frontLeft.setDirection(
                        frontLeftReversed
                                ? DcMotorSimple.Direction.REVERSE
                                : DcMotorSimple.Direction.FORWARD
                );
            }

            // B  →  Front Right
            if (gamepad1.b && !prevB) {
                frontRightReversed = !frontRightReversed;
                frontRight.setDirection(
                        frontRightReversed
                                ? DcMotorSimple.Direction.REVERSE
                                : DcMotorSimple.Direction.FORWARD
                );
            }

            // X  →  Back Left
            if (gamepad1.x && !prevX) {
                backLeftReversed = !backLeftReversed;
                backLeft.setDirection(
                        backLeftReversed
                                ? DcMotorSimple.Direction.REVERSE
                                : DcMotorSimple.Direction.FORWARD
                );
            }

            // Y  →  Back Right
            if (gamepad1.y && !prevY) {
                backRightReversed = !backRightReversed;
                backRight.setDirection(
                        backRightReversed
                                ? DcMotorSimple.Direction.REVERSE
                                : DcMotorSimple.Direction.FORWARD
                );
            }

            // Save button states for next loop iteration
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            // --- Run all motors at 0.1 power ---
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            backLeft.setPower(0.1);
            backRight.setPower(0.1);

            // --- Telemetry ---
            telemetry.addLine("=== Motor Direction Test ===");
            telemetry.addLine("All motors running at 0.1 power");
            telemetry.addLine("");
            telemetry.addData("Front Left  (A)", frontLeftReversed  ? "REVERSED" : "FORWARD");
            telemetry.addData("Front Right (B)", frontRightReversed ? "REVERSED" : "FORWARD");
            telemetry.addData("Back Left   (X)", backLeftReversed   ? "REVERSED" : "FORWARD");
            telemetry.addData("Back Right  (Y)", backRightReversed  ? "REVERSED" : "FORWARD");
            telemetry.update();
        }

        // Stop all motors when OpMode ends
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}