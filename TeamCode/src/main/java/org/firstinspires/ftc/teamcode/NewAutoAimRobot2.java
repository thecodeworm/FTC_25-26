package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp(name="NewAutoAimRobot2", group="Competition")
public class NewAutoAimRobot2 extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter system
    private DcMotor shooter;
    private DcMotor roller;
    private Servo kicker;

    // Sort wheel / Spindexer
    private ServoImplEx sortWheel;
    private ColorSensor cs;

    // Turret control
    private Servo turretYaw;
    private DcMotor turretPitch;

    // Vision - Limelight 3A
    private Limelight3A limelight;

    // ---------- STORAGE POSITIONS ----------
    private static final double SLOT_0_POSITION = 0.0;
    private static final double SLOT_1_POSITION = 0.38;
    private static final double SLOT_2_POSITION = 0.78;

    // ---------- SHOOTING POSITIONS ----------
    private static final double SHOOT_POSITION_0 = 0.57; 
    private static final double SHOOT_POSITION_1 = 0.96;
    private static final double SHOOT_POSITION_2 = 0.1700;

    // ---------- KICKER POSITIONS ----------
    private static final double KICKER_OPEN = 0;
    private static final double KICKER_CLOSED = 0.38;

    // ---------- TURRET YAW SERVO TUNING ----------
    private static final double YAW_CENTER = 0.5;
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double YAW_SERVO_GAIN = 0.01;
    private static final double MANUAL_YAW_STEP = 0.01;

    // ---------- AUTO-AIM CONSTANTS ----------
    private static final double TURRET_PITCH_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double TY_GAIN = 0.02;

    // ---------- INTAKE CONSTANTS ----------
    private static final int SHOOT_DELAY = 375;
    private static final int AUTO_ROTATE_DELAY = 450;
    private static final int COLOR_THRESHOLD = 80;
    private static final int MAX_BALLS = 3;
    private static final int DETECTION_COOLDOWN = 600;
    private static final double INTAKE_POWER = 0.35;
    private static final int SERVO_MOVE_DELAY = 750;

    // ---------- BALL STORAGE ----------
    private String[] ballSlots = new String[3];
    private int currentSlot = 0;
    private int ballCount = 0;

    // ---------- TARGET PATTERN ----------
    private String[] targetPattern = null;
    private int detectedAprilTagId = -1;
    private boolean motifDetected = false;

    // ---------- STATE VARIABLES ----------
    private double yawPosition = YAW_CENTER;
    private boolean autoAimEnabled = false;
    private boolean isShooting = false;
    private boolean ballDetectedInSlot = false;
    private boolean intakeEnabled = true;
    private boolean inCooldown = false;
    private boolean shooterEnabled = false;

    // ---------- TOGGLE STATES ----------
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;
    private boolean lastDpadDownState = false;

    // ---------- TIMERS ----------
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime cooldownTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            // Toggle auto-aim with A button
            if (gamepad1.a && !lastAState) {
                autoAimEnabled = !autoAimEnabled;
                telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
                telemetry.update();
            }
            lastAState = gamepad1.a;

            // Auto-aim or manual turret control
            if (autoAimEnabled) {
                autoAim();
            } else {
                manualTurretControl();
            }

            // Control intake motor with right trigger
            controlRoller();

            // Continuously monitor color sensor and auto-rotate
            if (!isShooting && intakeEnabled) {
                autoDetectAndRotate();
            }

            // Scan AprilTag pattern with Y button
            if (gamepad1.y && !lastYState && !isShooting) {
                scanAprilTagPattern();
                sleep(225);
            }
            lastYState = gamepad1.y;

            // Toggle Shooter Motor with DPad Down
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooterEnabled = !shooterEnabled;
                if (shooterEnabled) {
                    shooter.setPower(0.8);
                    telemetry.addData("Shooter", "ENABLED");
                } else if (!isShooting) {
                    shooter.setPower(0);
                    telemetry.addData("Shooter", "DISABLED");
                }
                telemetry.update();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // Shoot sequence with B button - CHANGED: No longer requires pattern
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState && !isShooting) {
                if (ballCount < MAX_BALLS) {
                    telemetry.addData("Error", "Not enough balls! Have " + ballCount + ", need " + MAX_BALLS);
                    telemetry.update();
                    sleep(750);
                } else {
                    executeShootSequence();
                }
            }
            lastBState = currentBState;

            // Reset/Clear all balls with X button
            boolean currentXState = gamepad1.x;
            if (currentXState && !lastXState && !isShooting) {
                resetBalls();
            }
            lastXState = currentXState;
        }
    }

    private void initHardware() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Shooter system
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        roller = hardwareMap.get(DcMotor.class, "roller");
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.REVERSE);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_CLOSED);

        // Sort wheel with extended PWM range
        sortWheel = hardwareMap.get(ServoImplEx.class, "sortWheel");
        sortWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        sortWheel.setPosition(SLOT_0_POSITION);

        cs = hardwareMap.get(ColorSensor.class, "cs");

        // Turret control
        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretYaw.setPosition(YAW_CENTER);

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize ball slots as empty
        ballCount = 0;
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
    }

    private void mecanum(double drive, double strafe, double turn) {
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;

        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return;
        }

        // Find the GOAL tag (ID 20 for blue, ID 24 for red)
        LLResultTypes.FiducialResult target = fiducials.get(0);
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = (int) f.getFiducialId();
            if (id == 20 || id == 24) {
                target = f;
                break;
            }
        }

        double tx = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();
        double ta = target.getTargetArea();

        double estimatedDistance = (0.55 / ta)-12.2047;

        // YAW SERVO CONTROL
        yawPosition += tx * YAW_SERVO_GAIN;
        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // PITCH MOTOR CONTROL
        double pitchPower = Math.max(-TURRET_PITCH_SPEED,
                Math.min(TURRET_PITCH_SPEED, -ty * TY_GAIN));
        turretPitch.setPower(pitchPower);

        // SHOOTER POWER - only set if shooter is enabled and not already at the right power
        if (shooterEnabled && !isShooting) {
            double targetPower = calculateShooterPower(estimatedDistance);
            shooter.setPower(targetPower);
        }
    }

    private double calculateShooterPower(double estimatedDistance) {
        double minPower = 0.57;
        double maxPower = 0.8;
        double minDist = 14;
        double maxDist = 144;

        estimatedDistance = Math.max(minDist, Math.min(maxDist, estimatedDistance));

        return minPower + (estimatedDistance - minDist) * (maxPower - minPower) / (maxDist - minDist);
    }

    private void manualTurretControl() {
        // Bumpers control yaw
        if (gamepad1.left_bumper) {
            yawPosition -= MANUAL_YAW_STEP;
        } else if (gamepad1.right_bumper) {
            yawPosition += MANUAL_YAW_STEP;
        }

        yawPosition = Math.max(YAW_MIN, Math.min(YAW_MAX, yawPosition));
        turretYaw.setPosition(yawPosition);

        // Triggers control pitch
        double pitchPower = 0;
        if (gamepad1.left_trigger > 0.1) {
            pitchPower = -gamepad1.left_trigger * TURRET_PITCH_SPEED;
        } else if (gamepad1.right_trigger > 0.1 && !intakeEnabled) {
            pitchPower = gamepad1.right_trigger * TURRET_PITCH_SPEED;
        }

        turretPitch.setPower(pitchPower);
    }

    private void controlRoller() {
        if (gamepad1.right_trigger > 0.1 && intakeEnabled && !isShooting) {
            roller.setPower(INTAKE_POWER);
        } else {
            roller.setPower(0);
        }
    }

    private void autoDetectAndRotate() {
        if (ballCount >= MAX_BALLS) {
            return;
        }

        if (inCooldown) {
            if (cooldownTimer.milliseconds() > DETECTION_COOLDOWN) {
                inCooldown = false;
            } else {
                return;
            }
        }

        int red = cs.red();
        int green = cs.green();
        int blue = cs.blue();

        boolean ballPresent = (red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD);

        if (ballPresent && !ballDetectedInSlot && !inCooldown) {
            String detectedColor = detectBallColor(red, green, blue);

            if (detectedColor != null) {
                ballSlots[currentSlot] = detectedColor;
                ballCount++;
                ballDetectedInSlot = true;
                detectionTimer.reset();

                telemetry.addData("Ball Detected", detectedColor + " in Slot " + currentSlot);
                telemetry.addData("Ball Count", ballCount + "/" + MAX_BALLS);
                telemetry.update();

                if (ballCount >= MAX_BALLS) {
                    intakeEnabled = false;
                    telemetry.addData("Status", "ALL BALLS LOADED");
                    telemetry.update();
                    sleep(375);
                }
            }
        }

        if (ballDetectedInSlot && detectionTimer.milliseconds() > AUTO_ROTATE_DELAY) {
            if (ballCount < MAX_BALLS) {
                rotateSortWheelToNextSlot();
                inCooldown = true;
                cooldownTimer.reset();
            }
            ballDetectedInSlot = false;
        }
    }

    private String detectBallColor(int red, int green, int blue) {
        if (green > red && green > blue && green > COLOR_THRESHOLD) {
            return "Green";
        } else if ((red > green && blue > green && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))
                || (red + blue > green * 2 && (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD))) {
            return "Purple";
        }
        return null;
    }

    private void rotateSortWheelToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;

        switch (currentSlot) {
            case 0:
                sortWheel.setPosition(SLOT_0_POSITION);
                break;
            case 1:
                sortWheel.setPosition(SLOT_1_POSITION);
                break;
            case 2:
                sortWheel.setPosition(SLOT_2_POSITION);
                break;
        }
    }

    private void resetBalls() {
        for (int i = 0; i < 3; i++) {
            ballSlots[i] = null;
        }
        ballCount = 0;
        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        currentSlot = 0;
        sortWheel.setPosition(SLOT_0_POSITION);

        telemetry.addData("Status", "All balls cleared");
        telemetry.update();
        sleep(750);
    }

    private void scanAprilTagPattern() {
        telemetry.addData("Status", "Scanning for AprilTag pattern...");
        telemetry.update();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No valid Limelight data");
            telemetry.update();
            sleep(750);
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("Error", "No AprilTags detected");
            telemetry.update();
            sleep(750);
            return;
        }

        // Find pattern tags (21, 22, 23)
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = (int) fiducial.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                target = fiducial;
                detectedAprilTagId = id;
                setTargetPattern(id);
                motifDetected = true;
                break;

            }
        }

        if (target == null) {
            telemetry.addData("Error", "No pattern tags (21, 22, 23) found");
            telemetry.update();
            sleep(750);
            return;
        }

        telemetry.addData("Success", "Pattern loaded from Tag " + detectedAprilTagId);
        telemetry.addData("Pattern", getPatternString());
        telemetry.update();
        sleep(1500);
    }

    private void setTargetPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21:
                targetPattern = new String[]{"Green", "Purple", "Purple"};
                break;
            case 22:
                targetPattern = new String[]{"Purple", "Green", "Purple"};
                break;
            case 23:
                targetPattern = new String[]{"Purple", "Purple", "Green"};
                break;
            default:
                targetPattern = null;
        }
    }

    private void executeShootSequence() {
        if (isShooting) {
            return;
        }

        isShooting = true;
        roller.setPower(0);
        intakeEnabled = false;
        inCooldown = true;

        telemetry.addData("Starting", "Shoot sequence...");
        telemetry.addData("Pattern Order", getPatternString());
        telemetry.update();
        sleep(1125);

        String[] availableBalls = new String[3];
        for (int i = 0; i < 3; i++) {
            availableBalls[i] = ballSlots[i];
        }

        int[] shootOrder = new int[3];
        boolean canMatchPattern = true;

        // Determine which slot contains each ball we need to shoot
        for (int patternIndex = 0; patternIndex < 3; patternIndex++) {
            String requiredColor = targetPattern[patternIndex];
            int slotWithRequiredBall = -1;

            for (int slot = 0; slot < 3; slot++) {
                if (availableBalls[slot] != null && availableBalls[slot].equals(requiredColor)) {
                    slotWithRequiredBall = slot;
                    availableBalls[slot] = null;
                    break;
                }
            }

            if (slotWithRequiredBall == -1) {
                canMatchPattern = false;
                break;
            }

            shootOrder[patternIndex] = slotWithRequiredBall;
        }

        if (!canMatchPattern) {
            telemetry.addData("WARNING", "Insufficient balls for pattern!");
            telemetry.addData("Failsafe", "Shooting in slot order: 0→1→2");
            telemetry.update();
            sleep(1500);

            shootOrder[0] = 0;
            shootOrder[1] = 1;
            shootOrder[2] = 2;
        }

        // Enable shooter
        shooter.setPower(0.8);
        sleep(1000);

        for (int shootIndex = 0; shootIndex < 3; shootIndex++) {
            telemetry.addData("===", "SHOOTING POSITION " + (shootIndex + 1) + " OF 3 ===");
            telemetry.update();
            sleep(375);

            int slotToShoot = shootOrder[shootIndex];
            String ballColor = ballSlots[slotToShoot];

            if (ballColor == null) {
                telemetry.addData("ERROR", "Slot " + slotToShoot + " is empty!");
                telemetry.update();
                sleep(2250);
                continue;
            }

            telemetry.addData("Shooting", ballColor + " from Slot " + slotToShoot);
            telemetry.update();
            sleep(750);

            // KEY FIX: Use the slot number to determine the shoot position
            double shootPosition = getShootPositionForSlot(slotToShoot);
            sortWheel.setPosition(shootPosition);
            sleep(SERVO_MOVE_DELAY * 2);

            // Kick the ball
            kicker.setPosition(KICKER_OPEN);
            telemetry.addData("FIRING!", ballColor);
            telemetry.update();
            sleep(SHOOT_DELAY);

            kicker.setPosition(KICKER_CLOSED);
            sleep(300);

            ballSlots[slotToShoot] = null;
            ballCount--;

            telemetry.addData("Complete", "Position " + (shootIndex + 1) + " fired!");
            telemetry.update();
            sleep(750);

            // Open kicker between rotations (except after the last shot)
            if (shootIndex < 2) {
                kicker.setPosition(KICKER_OPEN);
                telemetry.addData("Kicker", "Opening for next rotation...");
                telemetry.update();
                sleep(300);

                kicker.setPosition(KICKER_CLOSED);
                sleep(200);
            }
        }

        shooter.setPower(0);

        telemetry.addData("===", "SEQUENCE COMPLETE ===");
        telemetry.update();
        sleep(750);

        sortWheel.setPosition(SLOT_0_POSITION);
        currentSlot = 0;
        sleep(SERVO_MOVE_DELAY);

        intakeEnabled = true;
        inCooldown = false;
        ballDetectedInSlot = false;
        isShooting = false;

        telemetry.addData("SUCCESS!", "Ready to load new balls");
        telemetry.update();
        sleep(1500);
    }

    // NEW METHOD: Get shoot position based on slot number
    private double getShootPositionForSlot(int slotNumber) {
        switch (slotNumber) {
            case 0:
                return SHOOT_POSITION_0;
            case 1:
                return SHOOT_POSITION_1;
            case 2:
                return SHOOT_POSITION_2;
            default:
                return SHOOT_POSITION_0;
        }
    }

    private double getShootPositionForIndex(int shootIndex) {
        switch (shootIndex) {
            case 0:
                return SHOOT_POSITION_0;
            case 1:
                return SHOOT_POSITION_1;
            case 2:
                return SHOOT_POSITION_2;
            default:
                return SHOOT_POSITION_0;
        }
    }

    private String getPatternString() {
        if (targetPattern == null) return "None";
        return String.format("[%s, %s, %s]", targetPattern[0], targetPattern[1], targetPattern[2]);
    }
}