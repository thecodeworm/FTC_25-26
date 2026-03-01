package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name="AutoAimRobot_PID", group="Competition")
public class AutoAimRobot_PID extends LinearOpMode {

    //=== Drive ===
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    //=== Shooter System ===
    private DcMotor shooter;
    private DcMotor roller;
    private Servo kicker;

    //=== Turret ===
    private Servo turretYaw;          // Up/Down (servo)
    private DcMotor turretPitch;      // Left/Right (motor)

    //=== Vision ===
    private Limelight3A limelight;

    //=== Auto-Aim Control ===
    private boolean autoAimEnabled = false;

    //=== PID FOR TURRET ===
    private double yawKp = 0.015;   // servo vertical
    private double yawKd = 0.002;
    private double yawKi = 0.0004;

    private double pitchKp = 0.022; // motor horizontal
    private double pitchKd = 0.0012;
    private double pitchKi = 0.0003;

    private double yawIntegral = 0;
    private double yawPrevError = 0;

    private double pitchIntegral = 0;
    private double pitchPrevError = 0;

    private double yawServoPos = 0.5; // neutral servo
    private final double yawMin = 0.25;
    private final double yawMax = 0.58;

    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            //=== Drive ===
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            //=== Auto Aim Toggle ===
            if (gamepad1.a) {
                autoAimEnabled = true;
            }
            if (gamepad1.x) {
                autoAimEnabled = false;
                turretPitch.setPower(0);
            }

            if (autoAimEnabled) {
                autoAimUpdate();
            } else {
                manualTurretControl();
            }

            telemetry.addData("AutoAim", autoAimEnabled);
            telemetry.addData("YawPos", yawServoPos);
            telemetry.update();
        }
    }

    private void initHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        roller = hardwareMap.get(DcMotor.class, "roller");

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        yawServoPos = 0.5;
        turretYaw.setPosition(yawServoPos);
    }

    private void mecanum(double d, double s, double t) {
        double fl = d + s + t;
        double fr = d - s - t;
        double bl = d - s + t;
        double br = d + s - t;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void autoAimUpdate() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags.isEmpty()) return;

        LLResultTypes.FiducialResult target = tags.get(0);
        double tx = target.getTargetXDegrees(); // left/right
        double ty = target.getTargetYDegrees(); // up/down

        double dt = pidTimer.seconds();
        pidTimer.reset();

        //=== PITCH PID (motor, left/right) ===
        double pitchError = tx;
        pitchIntegral += pitchError * dt;
        double pitchDerivative = (pitchError - pitchPrevError) / dt;
        pitchPrevError = pitchError;

        double pitchCmd =
                pitchKp * pitchError +
                        pitchKi * pitchIntegral +
                        pitchKd * pitchDerivative;

        pitchCmd = Math.max(-0.6, Math.min(0.6, pitchCmd)); // clamp motor
        turretPitch.setPower(pitchCmd);

        //=== YAW PID (servo, up/down) ===
        double yawError = -ty; // invert because up is negative
        yawIntegral += yawError * dt;
        double yawDerivative = (yawError - yawPrevError) / dt;
        yawPrevError = yawError;

        double yawCmd =
                yawKp * yawError +
                        yawKi * yawIntegral +
                        yawKd * yawDerivative;

        yawServoPos += yawCmd;
        yawServoPos = Math.max(yawMin, Math.min(yawMax, yawServoPos));
        turretYaw.setPosition(yawServoPos);

        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("pitchCmd", pitchCmd);
        telemetry.addData("yawCmd", yawCmd);
    }

    private void manualTurretControl() {
        // bumpers control yaw
        if (gamepad1.left_bumper) yawServoPos -= 0.005;
        if (gamepad1.right_bumper) yawServoPos += 0.005;
        yawServoPos = Math.max(yawMin, Math.min(yawMax, yawServoPos));
        turretYaw.setPosition(yawServoPos);

        // triggers control pitch
        double pwr = 0;
        if (gamepad1.left_trigger > 0.1) pwr = -gamepad1.left_trigger * 0.4;
        if (gamepad1.right_trigger > 0.1) pwr = gamepad1.right_trigger * 0.4;
        turretPitch.setPower(pwr);
    }
}
