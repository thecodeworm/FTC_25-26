package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "autoaimdrive", group = "Test")
public class autoaimdrive extends LinearOpMode {

    // =========================
    // DRIVE MOTORS
    // =========================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter
    private DcMotor shooter;

    // Turret
    private ServoImplEx sortWheel;     // goBILDA 300° SERVO (FIXED)
    private DcMotor turretPitch;

    // Vision
    private Limelight3A limelight;

    // =========================
    // CONSTANTS
    // =========================
    private static final double TURRET_PITCH_SPEED = 0.6;

    private static final double TX_GAIN = 0.04;
    private static final double TY_GAIN = 0.04;

    // Sort wheel (300° servo)
    private static final double SORT_MIN = 0.03;   // soft limits
    private static final double SORT_MAX = 0.97;

    // Camera offset
    private static final double CAMERA_OFFSET_METERS = 0.0;

    // =========================
    // STATE
    // =========================
    private double sortPosition = 0.0;
    private boolean shooterOn = false;
    private boolean lastBState = false;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "B = Toggle Shooter");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // =========================
            // DRIVE CONTROLS
            // =========================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(clamp(y + x + rx, -1, 1));
            frontRight.setPower(clamp(y - x - rx, -1, 1));
            backLeft.setPower(clamp(y - x + rx, -1, 1));
            backRight.setPower(clamp(y + x - rx, -1, 1));

            // =========================
            // SHOOTER TOGGLE
            // =========================
            if (gamepad1.b && !lastBState) {
                shooterOn = !shooterOn;
            }
            lastBState = gamepad1.b;

            // =========================
            // AUTO AIM
            // =========================
            autoAim();

            updateTelemetry();
        }
    }

    // =========================
    // HARDWARE INIT
    // =========================
    private void initHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // ---------- SORT WHEEL SERVO (FIX) ----------
        sortWheel = hardwareMap.get(ServoImplEx.class, "sortWheel");
        sortWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        sortWheel.setPosition(sortPosition);

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // =========================
    // AUTO AIM LOGIC
    // =========================
    private void autoAim() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            shooter.setPower(shooterOn ? calculateShooterPower(50.0) : 0);
            turretPitch.setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            shooter.setPower(shooterOn ? calculateShooterPower(50.0) : 0);
            turretPitch.setPower(0);
            return;
        }

        LLResultTypes.FiducialResult target = fiducials.get(0);
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == 20 || f.getFiducialId() == 24) {
                target = f;
                break;
            }
        }

        double tx = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();
        double ta = target.getTargetArea();

        double estimatedDistance = 0.55 / ta;

        double distanceMeters = estimatedDistance * 0.0254;
        double offsetAngle = Math.toDegrees(Math.atan2(CAMERA_OFFSET_METERS, distanceMeters));
        double adjustedTx = tx - offsetAngle;

        // ---------- SORT WHEEL CONTROL ----------
        sortPosition += adjustedTx * TX_GAIN;
        sortPosition = clamp(sortPosition, SORT_MIN, SORT_MAX);
        sortWheel.setPosition(sortPosition);

        // ---------- PITCH ----------
        double pitchPower = clamp(-ty * TY_GAIN, -TURRET_PITCH_SPEED, TURRET_PITCH_SPEED);
        turretPitch.setPower(pitchPower);

        shooter.setPower(shooterOn ? calculateShooterPower(estimatedDistance) : 0);
    }

    // =========================
    // SHOOTER CURVE
    // =========================
    private double calculateShooterPower(double distanceInches) {

        double minPower = 0.57;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 140;

        distanceInches = clamp(distanceInches, minDist, maxDist);
        double normalized = (distanceInches - minDist) / (maxDist - minDist);

        double curve = -4.0 * Math.pow(normalized - 0.5, 2) + 1.0;
        return minPower +
                (maxPower - minPower) * normalized +
                0.07 * curve;
    }

    private void updateTelemetry() {
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Sort Pos", sortPosition);
        telemetry.addData("Runtime", runtime);
        telemetry.update();
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

