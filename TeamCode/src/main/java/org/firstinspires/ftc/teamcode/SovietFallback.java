package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="SimpleTele + PixelAlign Fallback", group="TeleOp")
public class SovietFallback extends LinearOpMode {

    private boolean shooterOn = false;
    private boolean shooterBPressed = false;

    private boolean servosOn = false;
    private boolean servoAPressed = false;

    @Override
    public void runOpMode() {

        // ---------------- MOTOR + SERVO MAP ----------------
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        CRServo speedServo1 = hardwareMap.get(CRServo.class, "speedServo1");
        CRServo torqueServo1 = hardwareMap.get(CRServo.class, "torqueServo1");
        CRServo speedServo2 = hardwareMap.get(CRServo.class, "speedServo2");
        CRServo torqueServo2 = hardwareMap.get(CRServo.class, "torqueServo2");
        CRServo torqueServo3 = hardwareMap.get(CRServo.class, "torqueServo3");
        CRServo torqueServo4 = hardwareMap.get(CRServo.class, "torqueServo4");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Press Play.");
        telemetry.update();
        waitForStart();

        // ================= MAIN LOOP ==================
        while (opModeIsActive()) {

            // ------------- DRIVETRAIN ----------------
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double rotate = gamepad2.right_stick_x;

            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                    Math.max(Math.abs(bl), Math.abs(br))));

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            fl *= 0.5;
            fr *= 0.5;
            bl *= 0.5;
            br *= 0.5;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ------------- SHOOTER TOGGLE ----------------
            if (gamepad2.b && !shooterBPressed) shooterOn = !shooterOn;
            shooterBPressed = gamepad2.b;
            shooter.setPower(shooterOn ? 1 : 0);

            // ------------- SERVO TOGGLE ----------------
            if (gamepad2.a && !servoAPressed) servosOn = !servosOn;
            servoAPressed = gamepad2.a;

            if (servosOn) {
                speedServo1.setPower(-1);
                torqueServo1.setPower(-1);
                torqueServo3.setPower(-1);

                speedServo2.setPower(1);
                torqueServo2.setPower(1);
                torqueServo4.setPower(1);
            } else {
                speedServo1.setPower(0);
                torqueServo1.setPower(0);
                torqueServo3.setPower(0);
                speedServo2.setPower(0);
                torqueServo2.setPower(0);
                torqueServo4.setPower(0);
            }

            // ------------- PIXEL-BASED AUTO-ALIGN (SIMULATED) ----------------
            if (gamepad1.x) {
                // Simulate tag pixel position for testing
                final int simulatedTagX = 400; // centered
                final int centerX = 400;
                double xError = simulatedTagX - centerX;
                double xTolerance = 20;

                if (Math.abs(xError) > xTolerance) {
                    double power = 0.002 * xError;
                    power = Math.max(Math.min(power, 0.4), -0.4);

                    frontLeft.setPower(-power);
                    backLeft.setPower(power);
                    frontRight.setPower(power);
                    backRight.setPower(-power);
                } else {
                    stopAll(frontLeft, frontRight, backLeft, backRight);
                }

                telemetry.addData("Simulated xError(px)", xError);
            }

            telemetry.update();
        }
    }

    private void stopAll(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
