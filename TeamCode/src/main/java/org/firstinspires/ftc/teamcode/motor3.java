package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class motor3 extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry.addData("Hardware: ", "Initialized");
        waitForStart();
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setPower(-0.6);
            frontRight.setPower(0.6);
            backLeft.setPower(-0.6);
            backRight.setPower(0.6);
            sleep(600);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
    }
}
