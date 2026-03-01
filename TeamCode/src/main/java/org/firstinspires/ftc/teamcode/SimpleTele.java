package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="SimpleTele", group="TeleOp")
public class SimpleTele extends LinearOpMode {

    private boolean shooterOn = false;
    private boolean bPressedLastLoop = false;
    private boolean servosOn = false;

    @Override
    public void runOpMode() {

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
        CRServo revServo1 = hardwareMap.get(CRServo.class, "revServo1");


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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;

            double rotate = gamepad2.right_stick_x;

            double flPower = drive + strafe + rotate;
            double frPower = drive - strafe - rotate;
            double blPower = drive - strafe + rotate;
            double brPower = drive + strafe - rotate;

            double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));

            if (maxPower > 0.25) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
            if (gamepad2.y)
                shooter.setPower(-1);
            boolean bPressedNow = gamepad2.b;
            if (bPressedNow && !bPressedLastLoop) {
                shooterOn = !shooterOn;
            }
            boolean aPressedNow = gamepad2.a;
            boolean aPressedLastLoop = false;
            if (aPressedNow && !aPressedLastLoop) {
                servosOn = !servosOn;
            }
            bPressedLastLoop = bPressedNow;

            if (shooterOn) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }
            if (servosOn) {
                speedServo1.setPower(-1);
                torqueServo1.setPower(-1);
                torqueServo3.setPower(-1);
                speedServo2.setPower(1);
                torqueServo2.setPower(1);
                torqueServo4.setPower(1);
                revServo1.setPower(1);
            } else {
                speedServo1.setPower(0);
                torqueServo1.setPower(0);
                torqueServo3.setPower(0);
                speedServo2.setPower(0);
                torqueServo2.setPower(0);
                torqueServo4.setPower(0);
                revServo1.setPower(0);
            }

        }
    }
}