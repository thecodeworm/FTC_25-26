package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Kicker Calibration", group="Calibration")
public class kickerCalibration extends LinearOpMode {

    private Servo kicker;
private DcMotor shooter;
    // Calibration state
    private int calibrationStep = 0;
    private double[] calibratedPositions = {0.0, 0.3333, 0.6666, 0.8333};
    private String[] positionNames = {"0° (Home)", "120°", "240°", "300°"};

    // Button tracking
    private boolean xPressed = false;
    private boolean aPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        kicker = hardwareMap.servo.get("kicker");

        // Start at first position
        kicker.setPosition(calibratedPositions[calibrationStep]);

        telemetry.addLine("=============================");
        telemetry.addLine("SPINDEXER CALIBRATION MODE");
        telemetry.addLine("=============================");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  X = Next Position");
        telemetry.addLine("  A = Save & Print Results");
        telemetry.addLine("  DPAD ← = Fine tune -0.01");
        telemetry.addLine("  DPAD → = Fine tune +0.01");
        telemetry.addLine("  DPAD ↓ = Coarse tune -0.05");
        telemetry.addLine("  DPAD ↑ = Coarse tune +0.05");
        telemetry.addLine("");
        telemetry.addLine("Ready to calibrate!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            // ===========================
            // FINE TUNING (±0.01)
            // ===========================
            if(gamepad1.dpad_right && !dpadRightPressed){
                double currentPos = kicker.getPosition();
                double newPos = Math.min(1.0, currentPos + 0.01);
                kicker.setPosition(newPos);
                calibratedPositions[calibrationStep] = newPos;
                dpadRightPressed = true;
            }
            if(!gamepad1.dpad_right) dpadRightPressed = false;

            if(gamepad1.dpad_left && !dpadLeftPressed){
                double currentPos = kicker.getPosition();
                double newPos = Math.max(0.0, currentPos - 0.01);
                kicker.setPosition(newPos);
                calibratedPositions[calibrationStep] = newPos;
                dpadLeftPressed = true;
            }
            if(!gamepad1.dpad_left) dpadLeftPressed = false;

            // ===========================
            // COARSE TUNING (±0.05)
            // ===========================
            if(gamepad1.dpad_up && !dpadUpPressed){
                double currentPos = kicker.getPosition();
                double newPos = Math.min(1.0, currentPos + 0.05);
                kicker.setPosition(newPos);
                calibratedPositions[calibrationStep] = newPos;
                dpadUpPressed = true;
            }
            if(!gamepad1.dpad_up) dpadUpPressed = false;

            if(gamepad1.dpad_down && !dpadDownPressed){
                double currentPos = kicker.getPosition();
                double newPos = Math.max(0.0, currentPos - 0.05);
                kicker.setPosition(newPos);
                calibratedPositions[calibrationStep] = newPos;
                dpadDownPressed = true;
            }
            if(!gamepad1.dpad_down) dpadDownPressed = false;

            // ===========================
            // NEXT POSITION (X)
            // ===========================
            if(gamepad1.x && !xPressed){
                calibrationStep++;
                if(calibrationStep >= calibratedPositions.length){
                    calibrationStep = 0; // Loop back to start
                }
                kicker.setPosition(calibratedPositions[calibrationStep]);
                xPressed = true;
            }
            if(!gamepad1.x) xPressed = false;

            // ===========================
            // SAVE & PRINT (A)
            // ===========================
            if(gamepad1.a && !aPressed){
                printCalibrationResults();
                aPressed = true;
            }
            if(!gamepad1.a) aPressed = false;

            // ===========================
            // TELEMETRY
            // ===========================
            telemetry.addLine("=============================");
            telemetry.addLine("CALIBRATING POSITION " + calibrationStep);
            telemetry.addLine("=============================");
            telemetry.addData("Position Name", positionNames[calibrationStep]);
            telemetry.addData("Current Value", "%.4f", kicker.getPosition());
            telemetry.addLine("");
            telemetry.addLine("Instructions:");
            telemetry.addLine("1. Align wheel perfectly for " + positionNames[calibrationStep]);
            telemetry.addLine("2. Use DPAD to adjust");
            telemetry.addLine("3. Press X for next position");
            telemetry.addLine("4. Press A when all done");
            telemetry.addLine("");
            telemetry.addLine("--- All Positions ---");
            for(int i = 0; i < calibratedPositions.length; i++){
                String marker = (i == calibrationStep) ? " <- CURRENT" : "";
                telemetry.addData(positionNames[i], "%.4f%s", calibratedPositions[i], marker);
            }
            telemetry.update();
        }
    }

    private void printCalibrationResults() {
        telemetry.clear();
        telemetry.addLine("=============================");
        telemetry.addLine("CALIBRATION COMPLETE!");
        telemetry.addLine("=============================");
        telemetry.addLine("");
        telemetry.addLine("Copy this into your code:");
        telemetry.addLine("");

        // Format the array nicely for copying
        StringBuilder arrayCode = new StringBuilder();
        arrayCode.append("private double[] spindexSteps = {");
        for(int i = 0; i < calibratedPositions.length; i++){
            arrayCode.append(String.format("%.4f", calibratedPositions[i]));
            if(i < calibratedPositions.length - 1){
                arrayCode.append(", ");
            }
        }
        arrayCode.append("};");

        telemetry.addLine(arrayCode.toString());
        telemetry.addLine("");
        telemetry.addLine("Individual values:");
        for(int i = 0; i < calibratedPositions.length; i++){
            telemetry.addData(positionNames[i], "%.4f", calibratedPositions[i]);
        }
        telemetry.addLine("");
        telemetry.addLine("Press STOP to exit");
        telemetry.update();
    }
}