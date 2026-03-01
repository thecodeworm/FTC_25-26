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

@TeleOp(name="AutoAim Test (Limelight)", group="Test")
public class AutoAimTest extends LinearOpMode {

    // Shooter motor
    private DcMotor shooter;

    // Turret control
    private Servo turretYaw;          // STANDARD SERVO
    private DcMotor turretPitch;       // 312 RPM motor with encoder

    // Vision
    private Limelight3A limelight;

    // ---------- CONSTANTS ----------
    private static final double TURRET_PITCH_SPEED = 0.3;

    private static final double TX_GAIN = 0.02;
    private static final double TY_GAIN = 0.02;

    // Servo yaw tuning
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.5828;
    private static final double YAW_MAX = 0.25;
    private static final double YAW_SERVO_GAIN = 0.01;

    // Camera offset (5cm to the right = 0.05m)
    private static final double CAMERA_OFFSET_METERS = 0.05;

    // ---------- STATE ----------
    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean lastAState = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A = Toggle Auto Aim");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Toggle auto-aim
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastAState = gamepad1.a;

            if (autoAimEnabled) {
                autoAim();
            } else {
                // When auto-aim is off, stop everything
                shooter.setPower(0);
                turretPitch.setPower(0);
            }

            updateTelemetry();
        }
    }

    private void initHardware() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");

        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretYaw.setPosition(YAW_CENTER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            shooter.setPower(0);
            turretPitch.setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            shooter.setPower(0);
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

        // Apply camera offset compensation (5cm to the right)
        // Convert distance to meters for calculation
        double distanceMeters = estimatedDistance * 0.0254; // inches to meters
        double offsetAngleDegrees = Math.toDegrees(Math.atan2(CAMERA_OFFSET_METERS, distanceMeters));

        // Adjust tx to compensate for camera being 5cm to the right
        double adjustedTx = tx - offsetAngleDegrees;

        // ---------- YAW SERVO CONTROL ----------
        yawPosition += adjustedTx * YAW_SERVO_GAIN;
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // ---------- PITCH MOTOR ----------
        double pitchPower = Math.max(
                -TURRET_PITCH_SPEED,
                Math.min(TURRET_PITCH_SPEED, -ty * TY_GAIN)
        );
        turretPitch.setPower(pitchPower);

        shooter.setPower(calculateShooterPower(estimatedDistance));

        telemetry.addData("TX (raw)", tx);
        telemetry.addData("TX (adjusted)", adjustedTx);
        telemetry.addData("Offset Angle", offsetAngleDegrees);
        telemetry.addData("TY", ty);
        telemetry.addData("Yaw Pos", yawPosition);
        telemetry.addData("Distance", estimatedDistance);
    }

    private double calculateShooterPower(double distanceInches) {
        double minPower = 0.55;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 140;

        distanceInches = Math.max(minDist, Math.min(maxDist, distanceInches));

        return minPower +
                (distanceInches - minDist) *
                        (maxPower - minPower) /
                        (maxDist - minDist);
    }

    private void updateTelemetry() {
        telemetry.addData("Auto Aim", autoAimEnabled ? "ON" : "OFF");
        telemetry.addData("Yaw Servo", yawPosition);
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }
}