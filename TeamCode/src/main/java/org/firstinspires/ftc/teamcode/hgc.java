package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp(name="AutoAim Test (Limelight)", group="Test")
public class hgc extends LinearOpMode {

    // Shooter motor
    private DcMotor shooter;

    // Turret control
    private CRServo turretYaw;
    private DcMotor turretPitch; // 312 RPM motor with encoder for pitch control

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // Constants
    private static final double TURRET_YAW_SPEED = 0.5; // CRServo speed
    private static final double TURRET_PITCH_SPEED = 0.3; // 312 RPM motor speed (adjust as needed)

    // Limelight processing
    private static final double TX_GAIN = 0.02; // Proportional gain for horizontal aiming
    private static final double TY_GAIN = 0.02; // Proportional gain for vertical aiming

    // Toggle state
    private boolean autoAimEnabled = false;
    private boolean lastAState = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Vision", "Limelight 3A");
        telemetry.addData("Controls", "A: Toggle Auto-Aim");
        telemetry.addData("Controls", "Left Bumper/Right Bumper: Manual Yaw");
        telemetry.addData("Controls", "Left Trigger/Right Trigger: Manual Pitch");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Toggle auto-aim with A button
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastAState = gamepad1.a;

            // Auto-aim or manual turret control
            if (autoAimEnabled) {
                autoAim();
            } else {
                manualTurretControl();
                // Manual shooter control when not in auto-aim
                if (gamepad1.right_trigger > 0.1) {
                    shooter.setPower(gamepad1.right_trigger);
                } else {
                    shooter.setPower(0);
                }
            }

            updateTelemetry();
        }
    }

    private void initHardware() {
        // Shooter motor
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Turret control
        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch"); // 312 RPM motor with encoder

        // Set turret pitch motor to brake mode for better control
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure encoder for turret pitch
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure Limelight
        limelight.pipelineSwitch(0); // Switch to AprilTag pipeline
        limelight.start(); // Start polling for data

        telemetry.addData("Limelight 3A", "Ready - Pipeline 0");
        telemetry.addData("Turret Pitch", "312 RPM Motor with Encoder");
        telemetry.update();
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turretYaw.setPower(0);
            turretPitch.setPower(0);
            shooter.setPower(0);
            telemetry.addData("Target", "No valid data from Limelight");
            return;
        }

        // Get AprilTag detections from Limelight
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            turretYaw.setPower(0);
            turretPitch.setPower(0);
            shooter.setPower(0);
            telemetry.addData("Target", "No AprilTags detected");
            return;
        }

        // Find the GOAL tag (ID 20 for blue, ID 24 for red)
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                target = fiducial;
                break;
            }
        }

        // If no GOAL tag found, use the first detected tag
        if (target == null) {
            target = fiducials.get(0);
        }

        // Get target angles from Limelight
        // tx = horizontal offset in degrees (-29.8 to 29.8)
        // ty = vertical offset in degrees (-24.85 to 24.85)
        double tx = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();

        // Get target area (percentage of image) for distance estimation
        double ta = target.getTargetArea();

        // Estimate distance based on target area
        // Larger area = closer distance
        // This formula should be calibrated based on your setup
        double estimatedDistance = 0.55/(ta); // Rough estimation

        // Calculate turret adjustments using proportional control
        double yawPower = Math.max(-TURRET_YAW_SPEED, Math.min(TURRET_YAW_SPEED, tx * TX_GAIN));
        double pitchPower = Math.max(-TURRET_PITCH_SPEED, Math.min(TURRET_PITCH_SPEED, -ty * TY_GAIN));

        turretYaw.setPower(yawPower);
        turretPitch.setPower(pitchPower);

        // Calculate shooter power based on distance
        double shooterPower = calculateShooterPower(estimatedDistance);
        shooter.setPower(shooterPower);

        telemetry.addData("Target", "Tag ID " + (int) target.getFiducialId());
        telemetry.addData("TX (Horizontal)", "%.2f deg", tx);
        telemetry.addData("TY (Vertical)", "%.2f deg", ty);
        telemetry.addData("TA (Area)", "%.2f %%", ta);
        telemetry.addData("Est. Distance", "%.1f in", estimatedDistance);
        telemetry.addData("Yaw Power", "%.2f", yawPower);
        telemetry.addData("Pitch Power", "%.2f", pitchPower);
        telemetry.addData("Pitch Encoder", turretPitch.getCurrentPosition());
        telemetry.addData("Shooter Power", "%.2f", shooterPower);
    }

    private double calculateShooterPower(double distance) {
        // Empirical formula - adjust based on testing
        // Closer = less power, farther = more power
        if (distance < 24) {
            return 0.5;
        } else if (distance < 48) {
            return 0.6 + (distance - 24) * 0.01;
        } else if (distance < 96) {
            return 0.8 + (distance - 48) * 0.005;
        } else {
            return 1.0;
        }
    }

    private void manualTurretControl() {
        // Bumpers control yaw (left/right)
        double yawPower = 0;
        if (gamepad1.left_bumper) {
            yawPower = -TURRET_YAW_SPEED;  // Rotate left
        } else if (gamepad1.right_bumper) {
            yawPower = TURRET_YAW_SPEED;   // Rotate right
        }

        // Triggers control pitch (up/down) - using 312 RPM motor with encoder
        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) {
            pitchPower = -gamepad1.left_trigger * TURRET_PITCH_SPEED;  // Pitch down
        } else if (gamepad1.right_trigger > 0.1) {
            pitchPower = gamepad1.right_trigger * TURRET_PITCH_SPEED;  // Pitch up
        }

        turretYaw.setPower(yawPower);
        turretPitch.setPower(pitchPower);

        telemetry.addData("Manual Yaw", "%.2f", yawPower);
        telemetry.addData("Manual Pitch", "%.2f", pitchPower);
        telemetry.addData("Pitch Encoder", turretPitch.getCurrentPosition());
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Vision System", "Limelight 3A");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED ✓" : "DISABLED");
        telemetry.addData("---", "---");
        telemetry.update();
    }
}