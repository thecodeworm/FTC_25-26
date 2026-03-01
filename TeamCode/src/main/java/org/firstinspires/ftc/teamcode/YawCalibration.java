package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="YawCalibration", group="Calibration")
public class YawCalibration extends LinearOpMode {

    private Servo turretYaw;
    private DcMotorEx shooter;

    // GoBilda Yellow Jacket 6000 RPM motor specifications
    private static final double TICKS_PER_REV = 103.6; // Yellow Jacket encoder ticks per revolution
    private static final double MAX_RPM = 6000.0;
    private static final double MAX_TICKS_PER_SECOND = (MAX_RPM / 60.0) * TICKS_PER_REV; // ~10,360 ticks/sec

    // Servo calibration state
    private int servoCalibrationStep = 0;
    private double[] calibratedPositions = {0.0, 0.3333, 0.6666, 0.8333};
    private String[] positionNames = {"0° (Home)", "120°", "240°", "300°"};

    // Motor calibration state (velocity in ticks per second)
    private int motorCalibrationStep = 0;
    private double[] calibratedVelocities = {
            0.0,                      // 0 RPM
            MAX_TICKS_PER_SECOND * 0.25,  // 1500 RPM
            MAX_TICKS_PER_SECOND * 0.5,   // 3000 RPM
            MAX_TICKS_PER_SECOND * 0.75,  // 4500 RPM
            MAX_TICKS_PER_SECOND          // 6000 RPM
    };
    private String[] velocityNames = {"0 RPM", "1500 RPM", "3000 RPM", "4500 RPM", "6000 RPM"};

    // Encoder tracking
    private int lastEncoderPosition = 0;
    private int currentEncoderPosition = 0;
    private double currentVelocity = 0.0;
    private double currentRPM = 0.0;
    private ElapsedTime encoderTimer = new ElapsedTime();

    // Button tracking - Servo (DPAD)
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Button tracking - Motor (Letters)
    private boolean yPressed = false;
    private boolean aPressed = false;
    private boolean xPressed = false;
    private boolean bPressed = false;

    // Button tracking - Encoder reset
    private boolean leftBumperPressed = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        turretYaw = hardwareMap.servo.get("turretYaw");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Configure motor for velocity control
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set velocity PID coefficients (may need tuning for your specific setup)
        // These are reasonable defaults for GoBilda motors
        shooter.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);

        // Start at first positions
        turretYaw.setPosition(calibratedPositions[servoCalibrationStep]);
        shooter.setVelocity(calibratedVelocities[motorCalibrationStep]);

        // Initialize encoder tracking
        lastEncoderPosition = shooter.getCurrentPosition();
        encoderTimer.reset();

        telemetry.addLine("=============================");
        telemetry.addLine("DUAL CALIBRATION MODE");
        telemetry.addLine("(Velocity Control)");
        telemetry.addLine("=============================");
        telemetry.addLine("");
        telemetry.addLine("SERVO Controls (DPAD):");
        telemetry.addLine("  DPAD ← = Fine tune -0.01");
        telemetry.addLine("  DPAD → = Fine tune +0.01");
        telemetry.addLine("  DPAD ↓ = Coarse tune -0.05");
        telemetry.addLine("  DPAD ↑ = Coarse tune +0.05");
        telemetry.addLine("");
        telemetry.addLine("MOTOR Controls (Letters):");
        telemetry.addLine("  Y = Fine tune +100 ticks/s");
        telemetry.addLine("  A = Fine tune -100 ticks/s");
        telemetry.addLine("  B = Coarse tune +500 ticks/s");
        telemetry.addLine("  X = Coarse tune -500 ticks/s");
        telemetry.addLine("  Left Bumper = Reset Encoder");
        telemetry.addLine("");
        telemetry.addLine("Ready to calibrate!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===========================
            // ENCODER TRACKING
            // ===========================
            currentEncoderPosition = shooter.getCurrentPosition();
            currentVelocity = shooter.getVelocity(); // Native ticks per second
            currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

            // Reset encoder
            if(gamepad1.left_bumper && !leftBumperPressed) {
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setVelocity(calibratedVelocities[motorCalibrationStep]);
                lastEncoderPosition = 0;
                currentEncoderPosition = 0;
                encoderTimer.reset();
                leftBumperPressed = true;
            }
            if(!gamepad1.left_bumper) leftBumperPressed = false;

            // ===========================
            // SERVO FINE TUNING (±0.01)
            // ===========================
            if(gamepad1.dpad_right && !dpadRightPressed){
                double currentPos = turretYaw.getPosition();
                double newPos = Math.min(1.0, currentPos + 0.01);
                turretYaw.setPosition(newPos);
                calibratedPositions[servoCalibrationStep] = newPos;
                dpadRightPressed = true;
            }
            if(!gamepad1.dpad_right) dpadRightPressed = false;

            if(gamepad1.dpad_left && !dpadLeftPressed){
                double currentPos = turretYaw.getPosition();
                double newPos = Math.max(0.0, currentPos - 0.01);
                turretYaw.setPosition(newPos);
                calibratedPositions[servoCalibrationStep] = newPos;
                dpadLeftPressed = true;
            }
            if(!gamepad1.dpad_left) dpadLeftPressed = false;

            // ===========================
            // SERVO COARSE TUNING (±0.05)
            // ===========================
            if(gamepad1.dpad_up && !dpadUpPressed){
                double currentPos = turretYaw.getPosition();
                double newPos = Math.min(1.0, currentPos + 0.05);
                turretYaw.setPosition(newPos);
                calibratedPositions[servoCalibrationStep] = newPos;
                dpadUpPressed = true;
            }
            if(!gamepad1.dpad_up) dpadUpPressed = false;

            if(gamepad1.dpad_down && !dpadDownPressed){
                double currentPos = turretYaw.getPosition();
                double newPos = Math.max(0.0, currentPos - 0.05);
                turretYaw.setPosition(newPos);
                calibratedPositions[servoCalibrationStep] = newPos;
                dpadDownPressed = true;
            }
            if(!gamepad1.dpad_down) dpadDownPressed = false;

            // ===========================
            // MOTOR FINE TUNING (±100 ticks/s ≈ ±58 RPM)
            // ===========================
            if(gamepad1.y && !yPressed){
                double currentVel = calibratedVelocities[motorCalibrationStep];
                double newVel = Math.min(MAX_TICKS_PER_SECOND, currentVel + 100);
                shooter.setVelocity(newVel);
                calibratedVelocities[motorCalibrationStep] = newVel;
                yPressed = true;
            }
            if(!gamepad1.y) yPressed = false;

            if(gamepad1.a && !aPressed){
                double currentVel = calibratedVelocities[motorCalibrationStep];
                double newVel = Math.max(0.0, currentVel - 100);
                shooter.setVelocity(newVel);
                calibratedVelocities[motorCalibrationStep] = newVel;
                aPressed = true;
            }
            if(!gamepad1.a) aPressed = false;

            // ===========================
            // MOTOR COARSE TUNING (±500 ticks/s ≈ ±290 RPM)
            // ===========================
            if(gamepad1.b && !bPressed){
                double currentVel = calibratedVelocities[motorCalibrationStep];
                double newVel = Math.min(MAX_TICKS_PER_SECOND, currentVel + 500);
                shooter.setVelocity(newVel);
                calibratedVelocities[motorCalibrationStep] = newVel;
                bPressed = true;
            }
            if(!gamepad1.b) bPressed = false;

            if(gamepad1.x && !xPressed){
                double currentVel = calibratedVelocities[motorCalibrationStep];
                double newVel = Math.max(0.0, currentVel - 500);
                shooter.setVelocity(newVel);
                calibratedVelocities[motorCalibrationStep] = newVel;
                xPressed = true;
            }
            if(!gamepad1.x) xPressed = false;

            // ===========================
            // TELEMETRY
            // ===========================
            telemetry.addLine("=============================");
            telemetry.addLine("DUAL CALIBRATION MODE");
            telemetry.addLine("(Velocity Control)");
            telemetry.addLine("=============================");
            telemetry.addLine("");

            telemetry.addLine("--- SERVO (Turret Yaw) ---");
            telemetry.addData("Step", servoCalibrationStep);
            telemetry.addData("Position Name", positionNames[servoCalibrationStep]);
            telemetry.addData("Current Value", "%.4f", turretYaw.getPosition());
            telemetry.addLine("");

            telemetry.addLine("--- MOTOR (Shooter) ---");
            telemetry.addData("Step", motorCalibrationStep);
            telemetry.addData("Target Name", velocityNames[motorCalibrationStep]);
            telemetry.addData("Target Velocity", "%.1f ticks/s", calibratedVelocities[motorCalibrationStep]);
            telemetry.addData("Target RPM", "%.1f", (calibratedVelocities[motorCalibrationStep] / TICKS_PER_REV) * 60.0);
            telemetry.addData("Actual Velocity", "%.1f ticks/s", currentVelocity);
            telemetry.addData("Actual RPM", "%.1f", currentRPM);
            telemetry.addData("Encoder Position", currentEncoderPosition);
            telemetry.addLine("");

            telemetry.addLine("Controls:");
            telemetry.addLine("DPAD = Adjust Servo");
            telemetry.addLine("Y/A = Motor ±100 ticks/s");
            telemetry.addLine("B/X = Motor ±500 ticks/s");
            telemetry.addLine("Left Bumper = Reset Encoder");
            telemetry.addLine("");

            telemetry.addLine("--- Servo Positions ---");
            for(int i = 0; i < calibratedPositions.length; i++){
                String marker = (i == servoCalibrationStep) ? " <- CURRENT" : "";
                telemetry.addData(positionNames[i], "%.4f%s", calibratedPositions[i], marker);
            }

            telemetry.addLine("");
            telemetry.addLine("--- Motor Velocities ---");
            for(int i = 0; i < calibratedVelocities.length; i++){
                String marker = (i == motorCalibrationStep) ? " <- CURRENT" : "";
                double rpm = (calibratedVelocities[i] / TICKS_PER_REV) * 60.0;
                telemetry.addData(velocityNames[i], "%.1f ticks/s (%.1f RPM)%s",
                        calibratedVelocities[i], rpm, marker);
            }

            telemetry.update();
        }
    }
}