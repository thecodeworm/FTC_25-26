package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumTurretTeleOp4_Fixed", group="Main")
public class MecanumTurretTeleOp4 extends LinearOpMode {


    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter / intake / sorter
    private DcMotor shooter, rollerMotor;
    private Servo sortWheel, kicker;

    // Turret CRServos
    private CRServo turretYaw, turretPitch;

    // Color sensor
    private ColorSensor cs;

    // Auto-aim state
    private boolean autoAimEnabled = true;
    private boolean autoAimButtonPressed = false;
    private boolean autoAimLocked = false;

    // Intake toggle state
    private boolean intakeOn = false;
    private boolean intakeButtonPressed = false;

    // Shooter state machine
    private enum ShooterState { IDLE, SPINUP, POP_UP, POP_DOWN, BETWEEN_SHOTS, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private int shooterShotsFired = 0;
    private final int SHOOTER_SHOT_COUNT = 3;
    private final double SPINUP_DURATION = 0.25;
    private final double POP_UP_DURATION = 0.30;
    private final double POP_DOWN_DURATION = 0.20;
    private ElapsedTime shooterTimer = new ElapsedTime();

    // Indexer (non-blocking pulses)
    private ElapsedTime indexerTimer = new ElapsedTime();
    private boolean indexerPulsePending = false;
    private final double INDEXER_PULSE_MS = 0.05;

    // Arm constants
    private final double ARM_REST_POS = 0.05;
    private final double ARM_UP_DELTA = 60.0 / 180.0;
    private final double ARM_UP_POS = clamp(ARM_REST_POS + ARM_UP_DELTA, 0.0, 1.0);
    private final double ARM_IDLE_DRIFT = 0.005;

    // Shooter power
    private final double SHOOTER_POWER_MIN = 0.6;
    private final double SHOOTER_POWER_MAX = 1.0;

    // Camera / auto-aim parameters
    private final double CAMERA_HFOV_DEG = 120.0;

    // Turret virtual angles (degrees)
    private double turretYawAngleDeg = 0.0;
    private double turretPitchAngleDeg = 0.0;
    private final double YAW_DEG_PER_SEC_AT_FULL_POWER = 120.0;
    private final double PITCH_DEG_PER_SEC_AT_FULL_POWER = 90.0;

    // Vision processor (user-provided)
    private MyTargetProcessor targetProcessor;

    // Loop timer
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // =========================
        // Hardware mapping
        // =========================
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        shooter = hardwareMap.dcMotor.get("shooter");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");

        sortWheel = hardwareMap.servo.get("sortWheel");
        kicker = hardwareMap.servo.get("kicker");

        turretYaw = hardwareMap.get(CRServo.class, "turretYaw");
        turretPitch = hardwareMap.get(CRServo.class, "turretPitch");

        cs = hardwareMap.get(ColorSensor.class, "cs");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);
        rollerMotor.setDirection(DcMotor.Direction.REVERSE);

        sortWheel.setPosition(0.5);
        kicker.setPosition(ARM_REST_POS);

        // =========================
        // ZERO TURRET AT INIT
        // =========================
        turretYaw.setPower(0.0);
        turretPitch.setPower(0.0);

        turretYawAngleDeg = 0.0;
        turretPitchAngleDeg = 0.0;

        targetProcessor = new MyTargetProcessor();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // =========================
            // DRIVE CONTROLS
            // =========================
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            frontLeft.setPower(clamp(y + x + rx, -1, 1));
            frontRight.setPower(clamp(y - x - rx, -1, 1));
            backLeft.setPower(clamp(y - x + rx, -1, 1));
            backRight.setPower(clamp(y + x - rx, -1, 1));

            // =========================
            // AUTO-AIM TOGGLE
            // =========================
            if (gamepad2.x && !autoAimButtonPressed) {
                autoAimEnabled = !autoAimEnabled;
                autoAimButtonPressed = true;
            }
            if (!gamepad2.x) autoAimButtonPressed = false;

            // =========================
            // INTAKE / SORTER
            // =========================x
            if (gamepad2.a && !intakeButtonPressed) {
                intakeOn = !intakeOn;
                intakeButtonPressed = true;
            }
            if (!gamepad2.a) intakeButtonPressed = false;

            rollerMotor.setPower(intakeOn ? 1.0 : 0.0);
            if (intakeOn) triggerIndexerPulse();

            // =========================
            // AUTO-AIM / TURRET
            // =========================
            double computedShooterPower = SHOOTER_POWER_MAX;
            autoAimLocked = false;

            if (autoAimEnabled && targetProcessor.targetDetected()) {
                computedShooterPower = autoAimTurretAndComputePowerWithParallax_NONBLOCKING(3.0, 120.0, dt);
                autoAimLocked = Math.abs(turretYaw.getPower()) < 0.05 && Math.abs(turretPitch.getPower()) < 0.05;
            } else {
                // Manual control
                double pitchPower = 0;
                if (gamepad2.left_bumper) pitchPower = 0.6;
                else if (gamepad2.right_bumper) pitchPower = -0.6;
                turretPitch.setPower(pitchPower);

                double yawPower = gamepad2.right_trigger - gamepad2.left_trigger;
                turretYaw.setPower(clamp(yawPower * 0.6, -0.6, 0.6));
            }

            // Stop turret if no input and auto-aim off
            if (!autoAimEnabled && gamepad2.left_bumper == false && gamepad2.right_bumper == false &&
                    gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                turretYaw.setPower(0);
                turretPitch.setPower(0);
            }

            // Update virtual turret angles
            turretYawAngleDeg += turretYaw.getPower() * YAW_DEG_PER_SEC_AT_FULL_POWER * dt;
            turretPitchAngleDeg += turretPitch.getPower() * PITCH_DEG_PER_SEC_AT_FULL_POWER * dt;
            turretYawAngleDeg = wrapAngleDeg(turretYawAngleDeg);

            // =========================
            // SHOOTER state machine trigger
            if (gamepad2.b && shooterState == ShooterState.IDLE && autoAimLocked) {
                shooterState = ShooterState.SPINUP;
                shooterShotsFired = 0;
                shooterTimer.reset();
            }

            updateShooterStateMachine();
            updateIndexerStateMachine();
            idleDriftArm();

            // =========================
            // TELEMETRY
            telemetry.addLine("=== Turret System ===");
            telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("AutoAim Locked", autoAimLocked ? "YES" : "NO");
            telemetry.addData("Turret Yaw Power", "%.2f", turretYaw.getPower());
            telemetry.addData("Turret Pitch Power", "%.2f", turretPitch.getPower());
            telemetry.addData("Turret Yaw Angle", "%.1f°", turretYawAngleDeg);
            telemetry.addData("Turret Pitch Angle", "%.1f°", turretPitchAngleDeg);

            telemetry.addLine("=== Intake / Shooter ===");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Shooter State", shooterState.toString());
            telemetry.addData("Shots Fired", shooterShotsFired);

            telemetry.addLine("=== Sorter / Arm ===");
            telemetry.addData("SortWheel Pos", "%.2f", sortWheel.getPosition());
            telemetry.addData("Kicker Pos", "%.2f", kicker.getPosition());

            telemetry.addLine("=== Motif ===");
            telemetry.addData("Order", String.join(", ", targetProcessor.getMotifOrder()));

            telemetry.update();
        }
    }

    // =========================
// NON-BLOCKING auto-aim
// =========================
    private double autoAimTurretAndComputePowerWithParallax_NONBLOCKING(double cameraOffsetInches, double distanceToTargetInches, double dt) {
        if (targetProcessor == null || !targetProcessor.targetDetected()) {
            turretYaw.setPower(0.0);
            turretPitch.setPower(0.0);
            return SHOOTER_POWER_MAX;
        }

        double targetX = targetProcessor.getTargetX();
        double targetY = targetProcessor.getTargetY();
        int width = targetProcessor.cameraWidth;
        int height = targetProcessor.cameraHeight;

        double centerX = width / 2.0;
        double centerY = height / 2.0;

        double angleX = (targetX - centerX) * CAMERA_HFOV_DEG / width;

        double hfovRad = Math.toRadians(CAMERA_HFOV_DEG);
        double aspect = (double) width / height;
        double vfovRad = 2.0 * Math.atan(Math.tan(hfovRad / 2.0) / aspect);
        double vfovDeg = Math.toDegrees(vfovRad);

        double angleY = (centerY - targetY) * vfovDeg / height;

        double parallaxDeg = Math.toDegrees(Math.atan(cameraOffsetInches / distanceToTargetInches));
        angleX += parallaxDeg;

        double KpYaw = 0.02;
        double KpPitch = 0.02;

        turretYaw.setPower(clamp(angleX * KpYaw, -0.6, 0.6));
        turretPitch.setPower(clamp(angleY * KpPitch, -0.6, 0.6));

        double normalized = clamp(Math.abs(angleX) / (CAMERA_HFOV_DEG / 2.0), 0.0, 1.0);
        return clamp(SHOOTER_POWER_MAX - normalized * (SHOOTER_POWER_MAX - SHOOTER_POWER_MIN), SHOOTER_POWER_MIN, SHOOTER_POWER_MAX);
    }

    // =========================
// INDEXER
// =========================
    private void triggerIndexerPulse() {
        if (!indexerPulsePending) {
            indexerPulsePending = true;
            indexerTimer.reset();
            sortWheel.setPosition(0.55);
        }
    }

    private void updateIndexerStateMachine() {
        if (indexerPulsePending && indexerTimer.seconds() >= INDEXER_PULSE_MS) {
            sortWheel.setPosition(0.5);
            indexerPulsePending = false;
        }
    }

    // =========================
// SHOOTER
// =========================
    private void updateShooterStateMachine() {
        switch (shooterState) {
            case IDLE:
            case DONE:
                shooter.setPower(0.0);
                kicker.setPosition(ARM_REST_POS);
                break;
            case SPINUP:
                shooter.setPower(SHOOTER_POWER_MAX);
                if (shooterTimer.seconds() >= SPINUP_DURATION) {
                    shooterState = ShooterState.POP_UP;
                    kicker.setPosition(ARM_UP_POS);
                    shooterTimer.reset();
                }
                break;
            case POP_UP:
                shooter.setPower(SHOOTER_POWER_MAX);
                if (shooterTimer.seconds() >= POP_UP_DURATION) {
                    kicker.setPosition(ARM_REST_POS);
                    shooterState = ShooterState.POP_DOWN;
                    shooterTimer.reset();
                }
                break;
            case POP_DOWN:
                shooter.setPower(SHOOTER_POWER_MAX);
                if (shooterTimer.seconds() >= POP_DOWN_DURATION) {
                    shooterShotsFired++;
                    if (shooterShotsFired >= SHOOTER_SHOT_COUNT) {
                        shooterState = ShooterState.DONE;
                        shooter.setPower(0.0);
                        kicker.setPosition(ARM_REST_POS);
                        shooterTimer.reset();
                    } else {
                        shooterState = ShooterState.BETWEEN_SHOTS;
                        shooterTimer.reset();
                    }
                }
                break;
            case BETWEEN_SHOTS:
                shooter.setPower(SHOOTER_POWER_MAX);
                if (shooterTimer.seconds() >= SPINUP_DURATION) {
                    shooterState = ShooterState.POP_UP;
                    kicker.setPosition(ARM_UP_POS);
                    shooterTimer.reset();
                }
                break;
        }
    }

    // =========================
// ARM idle drift
// =========================
    private void idleDriftArm() {
        double pos = kicker.getPosition();
        kicker.setPosition(clamp(pos - ARM_IDLE_DRIFT * 0.5, 0.0, 1.0));
    }

    // =========================
// UTILITIES
// =========================
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private static double wrapAngleDeg(double ang) {
        double a = ang % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

}
