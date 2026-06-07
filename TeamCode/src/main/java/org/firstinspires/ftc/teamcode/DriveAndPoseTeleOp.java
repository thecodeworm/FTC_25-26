package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive + Pose", group = "Competition")
public class DriveAndPoseTeleOp extends LinearOpMode {

    /* ==================== HARDWARE ==================== */
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;

    /* ==================== POSE ESTIMATOR ==================== */
    private RobotPoseEstimator poseEstimator;
    private StartingPositionSelector posSelector;

    /* ==================== MAIN ==================== */
    @Override
    public void runOpMode() {
        initHardware();

        // ── Pre-start: auto-init from Limelight, fallback to driver selection ──
        telemetry.addLine("Searching for AprilTags to auto-init pose...");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            long now = System.currentTimeMillis();
            poseEstimator.update(now);

            if (poseEstimator.isInitialized()) {
                telemetry.addLine("✓ Auto-init from Limelight!");
                telemetry.addData("X",       "%.0f mm", poseEstimator.getX());
                telemetry.addData("Y",       "%.0f mm", poseEstimator.getY());
                telemetry.addData("Heading", "%.1f°",   poseEstimator.getHeadingDegrees());
                telemetry.update();
                break;
            }

            if (poseEstimator.needsDriverSelection(now)) {
                posSelector.update(gamepad1);
                if (posSelector.isSelected()) {
                    poseEstimator.forceInitialize(
                            posSelector.getX(),
                            posSelector.getY(),
                            posSelector.getHeadingDeg()
                    );
                    telemetry.addLine("✓ Driver selected: " + posSelector.getName());
                    telemetry.update();
                    break;
                }
            }

            sleep(20);
        }

        // Failsafe: if driver hit START before anything resolved
        if (!poseEstimator.isInitialized()) {
            poseEstimator.forceInitialize(0, 0, 0);
            telemetry.addLine("WARNING: defaulting to (0, 0, 0°)");
            telemetry.update();
        }

        waitForStart();

        /* ==================== MAIN LOOP ==================== */
        while (opModeIsActive()) {

            // ── 1. Update pose estimator (always first) ──────────────────────
            poseEstimator.update(System.currentTimeMillis());

            // ── 2. Drive ─────────────────────────────────────────────────────
            double drive  = -gamepad1.right_stick_x;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.left_stick_y;
            mecanum(drive, strafe, turn);

            // ── 3. Telemetry ─────────────────────────────────────────────────
            telemetry.addLine("──── POSE ESTIMATOR ────");
            telemetry.addData("X (mm)",      "%.1f",  poseEstimator.getX());
            telemetry.addData("Y (mm)",      "%.1f",  poseEstimator.getY());
            telemetry.addData("Heading",     "%.2f°", poseEstimator.getHeadingDegrees());
            telemetry.addData("Uncertainty", "±%.0f mm", poseEstimator.getPositionUncertaintyMM());
            telemetry.addLine("──── DRIVE ────");
            telemetry.addData("Drive",  "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn",   "%.2f", turn);
            telemetry.update();
        }
    }

    /* ==================== HARDWARE INIT ==================== */
    private void initHardware() {
        // Drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pinpoint odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(0, 6, DistanceUnit.INCH); // ← match your PinpointFullSetup values
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD  // ← adjust if reversed in your test file
        );
        odo.resetPosAndIMU(); // robot must be still during init

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Pose estimator + driver fallback selector
        poseEstimator = new NewRobotPoseEstimator(odo, limelight);
        posSelector   = new StartingPositionSelector(telemetry);
    }

    /* ==================== MECANUM DRIVE ==================== */
    private void mecanum(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}