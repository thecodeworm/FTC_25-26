package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class motor2 extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry.addData("Hardware: ", "Initialized");
        waitForStart();
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

        while (opModeIsActive()) {

            revServo1.setPower(1);
            speedServo1.setPower(-1);
            torqueServo1.setPower(-1);
            torqueServo3.setPower(-1);
            speedServo2.setPower(1);
            torqueServo2.setPower(1);
            torqueServo4.setPower(1);

            frontLeft.setPower(0.6);
            frontRight.setPower(-0.6);
            backLeft.setPower(0.6);
            backRight.setPower(-0.6);
            sleep(400);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            shooter.setPower(0.9);
        }
       }
    }
