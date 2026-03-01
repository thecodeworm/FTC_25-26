package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Elite_Turret_Tuning", group="Tuning")
public class Elite_Turret_Tuning extends LinearOpMode {

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo turretYaw;
    private DcMotor turretPitch;
    private DcMotorEx shooter;
    private Limelight3A limelight;

    // Distance calculation constants
    private static final double CAMERA_HEIGHT = 15.375;
    private static final double TARGET_HEIGHT = 29.5;
    private static final double HEIGHT_DIFF = TARGET_HEIGHT - CAMERA_HEIGHT;
    private static final double CAMERA_MOUNT_ANGLE = 8.75;

    // Turret constants
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double TURRET_PITCH_MAX_SPEED = 1.0;
    private static final double TURRET_PITCH_MIN_SPEED = 0.3;
    private static final double TX_GAIN = 0.02;
    private static final double PITCH_DEADZONE = 1.0;

    // Adaptive power ramping for pitch
    private static final double STUCK_TIME_THRESHOLD = 500;
    private static final double POWER_RAMP_RATE = 0.1;
    private double currentPitchPower = TURRET_PITCH_MIN_SPEED;
    private ElapsedTime stuckTimer = new ElapsedTime();
    private double lastTxError = 0;
    private boolean wasStuck = false;

    // Data collection
    private ArrayList<DataPoint> dataPoints = new ArrayList<>();
    private ArrayList<DataPoint> successfulShots = new ArrayList<>();
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private int sampleCount = 0;
    private int successCount = 0;

    // Current readings from auto-aim
    private double currentDistance = 0;
    private double currentYawAngle = 0;
    private double currentShooterPower = 0;
    private double currentShooterVelocity = 0;
    private double currentYawServoPos = 0.5;
    private double filteredTX = 0;
    private double filteredDist = 0;

    // Manual adjustment controls
    private double shooterPowerAdjustment = 0.0; // Added to auto-calculated power
    private double yawAngleAdjustment = 0.0; // Added to auto-calculated angle
    private static final double POWER_STEP = 0.01;
    private static final double ANGLE_STEP = 0.005;

    private class DataPoint {
        double distance;
        double yawServoPosition;
        double yawAngleDegrees;
        double shooterPower;
        double shooterVelocity;
        double txValue;
        boolean wasSuccessful;
        long timestamp;

        DataPoint(double dist, double yawPos, double yawAngle, double power, double velocity, double tx, boolean success) {
            this.distance = dist;
            this.yawServoPosition = yawPos;
            this.yawAngleDegrees = yawAngle;
            this.shooterPower = power;
            this.shooterVelocity = velocity;
            this.txValue = tx;
            this.wasSuccessful = success;
            this.timestamp = System.currentTimeMillis();
        }
    }

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("=== ELITE TUNING - AUTO AIM ALWAYS ON ===");
        telemetry.addLine("Press Start to begin");
        telemetry.addLine("");
        telemetry.addLine("AUTO-AIM is ALWAYS active!");
        telemetry.addLine("Turret tracks target automatically");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Left Stick - Drive/Strafe");
        telemetry.addLine("Right Stick X - Turn");
        telemetry.addLine("");
        telemetry.addLine("Y - Increase Shooter Power");
        telemetry.addLine("A - Decrease Shooter Power");
        telemetry.addLine("X - Increase Yaw Angle");
        telemetry.addLine("B - Decrease Yaw Angle");
        telemetry.addLine("");
        telemetry.addLine("DPad Up - Record SUCCESS");
        telemetry.addLine("DPad Down - Record MISS");
        telemetry.addLine("DPad Right - Calculate Equations");
        telemetry.addLine("DPad Left - Export CSV");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            mecanum(drive, strafe, turn);

            // Manual power adjustment with Y and A
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                shooterPowerAdjustment += POWER_STEP;
                shooterPowerAdjustment = Math.min(0.2, shooterPowerAdjustment); // Max +0.2 adjustment
            }
            lastY = currentY;

            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                shooterPowerAdjustment -= POWER_STEP;
                shooterPowerAdjustment = Math.max(-0.2, shooterPowerAdjustment); // Max -0.2 adjustment
            }
            lastA = currentA;

            // Manual yaw angle adjustment with X and B
            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                yawAngleAdjustment += ANGLE_STEP;
                yawAngleAdjustment = Math.min(0.1, yawAngleAdjustment); // Max +0.1 adjustment
            }
            lastX = currentX;

            boolean currentB = gamepad1.b;
            if (currentB && !lastB) {
                yawAngleAdjustment -= ANGLE_STEP;
                yawAngleAdjustment = Math.max(-0.1, yawAngleAdjustment); // Max -0.1 adjustment
            }
            lastB = currentB;

            // AUTO-AIM (always running)
            autoAimLoop();

            // Record SUCCESSFUL shot with DPad Up
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                recordDataPoint(true);
            }
            lastDpadUp = currentDpadUp;

            // Record MISSED shot with DPad Down
            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                recordDataPoint(false);
            }
            lastDpadDown = currentDpadDown;

            // Calculate equations with DPad Right
            boolean currentDpadRight = gamepad1.dpad_right;
            if (currentDpadRight && !lastDpadRight) {
                analyzeData();
            }
            lastDpadRight = currentDpadRight;

            // Export CSV with DPad Left
            boolean currentDpadLeft = gamepad1.dpad_left;
            if (currentDpadLeft && !lastDpadLeft) {
                exportToCSV();
            }
            lastDpadLeft = currentDpadLeft;

            // Display telemetry
            displayTelemetry();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretYaw.setPosition(0.5);

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
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

    private void autoAimLoop() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            shooter.setPower(0);
            turretPitch.setPower(0);
            currentDistance = 0;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            shooter.setPower(0);
            turretPitch.setPower(0);
            currentDistance = 0;
            return;
        }

        // Find GOAL tag (ID 20 blue, ID 24 red)
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

        // Filter TX
        filteredTX = kalman(filteredTX, tx, 0.7);

        // Calculate distance
        double estimatedDistance = calculateDistance(target, ty);
        filteredDist = kalman(filteredDist, estimatedDistance, 0.65);
        currentDistance = filteredDist;

        // Calculate base shooter power from distance
        double baseShooterPower = calculateShooterPower(filteredDist);
        currentShooterPower = baseShooterPower + shooterPowerAdjustment;
        currentShooterPower = Math.max(0.0, Math.min(1.0, currentShooterPower));

        // Calculate base yaw angle from distance
        double baseYawAngle = calculateYawAngle(filteredDist);
        currentYawAngle = baseYawAngle + yawAngleAdjustment;

        // Convert to servo position
        currentYawServoPos = angleToServoPosition(currentYawAngle);
        currentYawServoPos = Math.max(YAW_MIN, Math.min(YAW_MAX, currentYawServoPos));
        turretYaw.setPosition(currentYawServoPos);

        // Pitch motor control with adaptive ramping
        double pitchPower = 0;
        if (Math.abs(filteredTX) > PITCH_DEADZONE) {
            boolean isStuck = Math.abs(filteredTX - lastTxError) < 0.5;

            if (isStuck && !wasStuck) {
                stuckTimer.reset();
                wasStuck = true;
            } else if (isStuck && stuckTimer.milliseconds() > STUCK_TIME_THRESHOLD) {
                currentPitchPower = Math.min(TURRET_PITCH_MAX_SPEED,
                        currentPitchPower + POWER_RAMP_RATE);
                stuckTimer.reset();
            } else if (!isStuck) {
                currentPitchPower = TURRET_PITCH_MIN_SPEED;
                wasStuck = false;
            }

            double proportionalPower = filteredTX * TX_GAIN;
            double powerMagnitude = Math.min(currentPitchPower, Math.abs(proportionalPower));
            pitchPower = (filteredTX > 0) ? powerMagnitude : -powerMagnitude;
            lastTxError = filteredTX;
        } else {
            currentPitchPower = TURRET_PITCH_MIN_SPEED;
            wasStuck = false;
        }

        turretPitch.setPower(pitchPower);
        sleep(150);
        turretPitch.setPower(0);

        // Set shooter power
        shooter.setPower(currentShooterPower);
        currentShooterVelocity = shooter.getVelocity();
    }

    private double calculateDistance(LLResultTypes.FiducialResult target, double ty) {
        if (target.getCameraPoseTargetSpace() != null) {
            Position pos = target.getCameraPoseTargetSpace().getPosition();
            double x = pos.x * 39.3701;
            double z = pos.z * 39.3701;
            return Math.sqrt(x * x + z * z);
        } else {
            double totalAngle = CAMERA_MOUNT_ANGLE + ty;
            double angleRadians = Math.toRadians(totalAngle);
            return HEIGHT_DIFF / Math.tan(angleRadians);
        }
    }

    private double calculateYawAngle(double distanceInches) {
        return 1.51 * Math.pow(0.97, distanceInches);
    }

    private double angleToServoPosition(double angle) {
        double maxAngle = calculateYawAngle(1.0);
        double minAngle = calculateYawAngle(300.0);
        double normalized = (angle - minAngle) / (maxAngle - minAngle);
        return YAW_MIN + normalized * (YAW_MAX - YAW_MIN);
    }

    private double calculateShooterPower(double distanceInches) {
        double power = 0.52 + ((Math.pow(distanceInches, distanceInches * 0.004)) / 100);
        return Math.max(0.50, Math.min(0.85, power));
    }

    private double kalman(double oldVal, double newVal, double k) {
        return (oldVal * (1 - k)) + (newVal * k);
    }

    private void recordDataPoint(boolean wasSuccessful) {
        if (currentDistance < 1.0) {
            telemetry.addData("Error", "No valid target!");
            telemetry.update();
            sleep(500);
            return;
        }

        DataPoint dp = new DataPoint(
                currentDistance,
                currentYawServoPos,
                currentYawAngle,
                currentShooterPower,
                currentShooterVelocity,
                filteredTX,
                wasSuccessful
        );

        dataPoints.add(dp);
        if (wasSuccessful) {
            successfulShots.add(dp);
            successCount++;
        }
        sampleCount++;

        telemetry.clear();
        telemetry.addData("✓ Recorded", wasSuccessful ? "SUCCESS" : "MISS");
        telemetry.addData("Sample", "#" + sampleCount);
        telemetry.addData("Distance", "%.1f in", currentDistance);
        telemetry.addData("Yaw Angle", "%.2f°", currentYawAngle);
        telemetry.addData("Shooter Power", "%.3f", currentShooterPower);
        telemetry.addData("Power Adj", "%+.3f", shooterPowerAdjustment);
        telemetry.addData("Angle Adj", "%+.3f", yawAngleAdjustment);
        telemetry.addData("Success Rate", "%.1f%%", (successCount * 100.0 / sampleCount));
        telemetry.update();
        sleep(1000);
    }

    private void analyzeData() {
        if (successfulShots.size() < 3) {
            telemetry.addData("Error", "Need at least 3 SUCCESSFUL shots!");
            telemetry.addData("Current", successfulShots.size() + " successes");
            telemetry.update();
            sleep(1000);
            return;
        }

        telemetry.clear();
        telemetry.addLine("=== ANALYZING DATA ===");
        telemetry.update();
        sleep(500);

        // Remove outliers
        ArrayList<DataPoint> cleanedData = removeOutliers(successfulShots);

        // Calculate regressions
        ExponentialFit yawExp = calculateExponentialRegression(cleanedData, true);
        PowerFit shooterPower = calculatePowerRegression(cleanedData);
        ExponentialFit shooterExp = calculateExponentialRegression(cleanedData, false);

        // Generate lookup tables
        InterpolationTable yawLUT = generateLookupTable(cleanedData, true);
        InterpolationTable shooterLUT = generateLookupTable(cleanedData, false);

        // Display results
        displayAnalysisResults(yawExp, shooterPower, shooterExp, yawLUT, shooterLUT, cleanedData.size());
    }

    private ArrayList<DataPoint> removeOutliers(ArrayList<DataPoint> data) {
        ArrayList<Double> powers = new ArrayList<>();
        for (DataPoint dp : data) {
            powers.add(dp.shooterPower);
        }
        Collections.sort(powers);

        int n = powers.size();
        double q1 = powers.get(n / 4);
        double q3 = powers.get(3 * n / 4);
        double iqr = q3 - q1;
        double lowerBound = q1 - 1.5 * iqr;
        double upperBound = q3 + 1.5 * iqr;

        ArrayList<DataPoint> cleaned = new ArrayList<>();
        int removed = 0;
        for (DataPoint dp : data) {
            if (dp.shooterPower >= lowerBound && dp.shooterPower <= upperBound) {
                cleaned.add(dp);
            } else {
                removed++;
            }
        }

        telemetry.addData("Outliers Removed", removed);
        telemetry.update();
        sleep(500);

        return cleaned;
    }

    private class InterpolationTable {
        ArrayList<Double> distances;
        ArrayList<Double> values;

        InterpolationTable() {
            distances = new ArrayList<>();
            values = new ArrayList<>();
        }

        void addPoint(double dist, double val) {
            distances.add(dist);
            values.add(val);
        }

        String toCode(String varName) {
            StringBuilder sb = new StringBuilder();
            sb.append("// Interpolation Lookup Table for " + varName + "\n");
            sb.append("private static final double[][] " + varName + "_LUT = {\n");
            for (int i = 0; i < distances.size(); i++) {
                sb.append(String.format("    {%.1f, %.4f}", distances.get(i), values.get(i)));
                if (i < distances.size() - 1) sb.append(",");
                sb.append("\n");
            }
            sb.append("};\n");
            return sb.toString();
        }
    }

    private InterpolationTable generateLookupTable(ArrayList<DataPoint> data, boolean useAngle) {
        ArrayList<DataPoint> sorted = new ArrayList<>(data);
        Collections.sort(sorted, new Comparator<DataPoint>() {
            @Override
            public int compare(DataPoint a, DataPoint b) {
                return Double.compare(a.distance, b.distance);
            }
        });

        InterpolationTable table = new InterpolationTable();
        double binSize = 12.0;
        double currentBinStart = Math.floor(sorted.get(0).distance / binSize) * binSize;
        double maxDist = sorted.get(sorted.size() - 1).distance;

        while (currentBinStart <= maxDist) {
            double binEnd = currentBinStart + binSize;
            double sumValues = 0;
            int count = 0;

            for (DataPoint dp : sorted) {
                if (dp.distance >= currentBinStart && dp.distance < binEnd) {
                    sumValues += useAngle ? dp.yawAngleDegrees : dp.shooterPower;
                    count++;
                }
            }

            if (count > 0) {
                double avgValue = sumValues / count;
                double binCenter = currentBinStart + binSize / 2;
                table.addPoint(binCenter, avgValue);
            }

            currentBinStart += binSize;
        }

        return table;
    }

    private class ExponentialFit {
        double a, b, rSquared;
        ExponentialFit(double a, double b, double rSquared) {
            this.a = a;
            this.b = b;
            this.rSquared = rSquared;
        }
    }

    private class PowerFit {
        double a, b, c, rSquared;
        PowerFit(double a, double b, double c, double rSquared) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.rSquared = rSquared;
        }
    }

    private ExponentialFit calculateExponentialRegression(ArrayList<DataPoint> data, boolean useAngle) {
        int n = data.size();
        double sumX = 0, sumLnY = 0, sumXLnY = 0, sumXX = 0;

        for (DataPoint dp : data) {
            double x = dp.distance;
            double y = useAngle ? dp.yawAngleDegrees : dp.shooterPower;

            if (y <= 0) y = 0.001;

            double lnY = Math.log(y);
            sumX += x;
            sumLnY += lnY;
            sumXLnY += x * lnY;
            sumXX += x * x;
        }

        double lnB = (n * sumXLnY - sumX * sumLnY) / (n * sumXX - sumX * sumX);
        double lnA = (sumLnY - lnB * sumX) / n;

        double a = Math.exp(lnA);
        double b = Math.exp(lnB);

        double meanY = 0;
        for (DataPoint dp : data) {
            double y = useAngle ? dp.yawAngleDegrees : dp.shooterPower;
            meanY += y;
        }
        meanY /= n;

        double ssRes = 0, ssTot = 0;
        for (DataPoint dp : data) {
            double x = dp.distance;
            double y = useAngle ? dp.yawAngleDegrees : dp.shooterPower;
            double predicted = a * Math.pow(b, x);

            ssRes += Math.pow(y - predicted, 2);
            ssTot += Math.pow(y - meanY, 2);
        }

        double rSquared = 1 - (ssRes / ssTot);
        return new ExponentialFit(a, b, rSquared);
    }

    private PowerFit calculatePowerRegression(ArrayList<DataPoint> data) {
        int n = data.size();
        double bestA = 0.52, bestB = 0.004, bestC = 100.0, bestError = Double.MAX_VALUE;

        for (double testA = 0.5; testA <= 0.65; testA += 0.01) {
            for (double testB = 0.002; testB <= 0.008; testB += 0.0005) {
                for (double testC = 80; testC <= 150; testC += 10) {
                    double error = 0;

                    for (DataPoint dp : data) {
                        double x = dp.distance;
                        double y = dp.shooterPower;
                        double predicted = testA + (Math.pow(x, x * testB) / testC);
                        error += Math.pow(y - predicted, 2);
                    }

                    if (error < bestError) {
                        bestError = error;
                        bestA = testA;
                        bestB = testB;
                        bestC = testC;
                    }
                }
            }
        }

        double meanY = 0;
        for (DataPoint dp : data) {
            meanY += dp.shooterPower;
        }
        meanY /= n;

        double ssRes = 0, ssTot = 0;
        for (DataPoint dp : data) {
            double x = dp.distance;
            double y = dp.shooterPower;
            double predicted = bestA + (Math.pow(x, x * bestB) / bestC);

            ssRes += Math.pow(y - predicted, 2);
            ssTot += Math.pow(y - meanY, 2);
        }

        double rSquared = 1 - (ssRes / ssTot);
        return new PowerFit(bestA, bestB, bestC, rSquared);
    }

    private void displayAnalysisResults(ExponentialFit yawExp, PowerFit shooterPower,
                                        ExponentialFit shooterExp, InterpolationTable yawLUT,
                                        InterpolationTable shooterLUT, int cleanedSize) {
        telemetry.clear();
        telemetry.addLine("=== ELITE ANALYSIS ===");
        telemetry.addLine("");

        telemetry.addData("Total Samples", sampleCount);
        telemetry.addData("Successful", successCount);
        telemetry.addData("Success Rate", "%.1f%%", (successCount * 100.0 / sampleCount));
        telemetry.addData("After Cleaning", cleanedSize);
        telemetry.addLine("");

        telemetry.addLine("--- YAW ANGLE ---");
        telemetry.addData("Exponential", String.format("%.5f*%.6f^x", yawExp.a, yawExp.b));
        telemetry.addData("R²", String.format("%.4f", yawExp.rSquared));
        telemetry.addData("LUT Points", yawLUT.distances.size());
        telemetry.addLine("");

        telemetry.addLine("--- SHOOTER POWER ---");
        telemetry.addData("Power", String.format("%.2f+(x^(x*%.5f))/%.0f",
                shooterPower.a, shooterPower.b, shooterPower.c));
        telemetry.addData("R²", String.format("%.4f", shooterPower.rSquared));
        telemetry.addData("Exponential", String.format("%.5f*%.6f^x",
                shooterExp.a, shooterExp.b));
        telemetry.addData("R²", String.format("%.4f", shooterExp.rSquared));
        telemetry.addData("LUT Points", shooterLUT.distances.size());
        telemetry.addLine("");

        if (shooterPower.rSquared > 0.95 && yawExp.rSquared > 0.95) {
            telemetry.addLine("✓ Use regression equations");
        } else if (shooterLUT.distances.size() >= 5) {
            telemetry.addLine("✓ Use Interpolation LUT");
        } else {
            telemetry.addLine("⚠ Collect more data!");
        }

        telemetry.update();

        while (opModeIsActive() && !gamepad1.start) {
            sleep(50);
        }
    }

    private void exportToCSV() {
        try {
            String filename = "/sdcard/FIRST/turret_" + System.currentTimeMillis() + ".csv";
            FileWriter writer = new FileWriter(filename);

            writer.write("Distance,YawPos,YawAngle,Power,Velocity,TX,Success\n");

            for (DataPoint dp : dataPoints) {
                writer.write(String.format("%.2f,%.4f,%.2f,%.4f,%.0f,%.2f,%s\n",
                        dp.distance, dp.yawServoPosition, dp.yawAngleDegrees,
                        dp.shooterPower, dp.shooterVelocity, dp.txValue,
                        dp.wasSuccessful ? "1" : "0"));
            }

            writer.close();

            telemetry.addData("✓ Exported", filename);
            telemetry.update();
            sleep(1500);
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to export CSV");
            telemetry.update();
            sleep(1000);
        }
    }

    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("=== AUTO-AIM TUNING (ALWAYS ON) ===");
        telemetry.addData("Samples", sampleCount + " (" + successCount + " success)");
        if (sampleCount > 0) {
            telemetry.addData("Success Rate", "%.1f%%", (successCount * 100.0 / sampleCount));
        }
        telemetry.addLine("");

        telemetry.addLine("--- Auto-Aim Status ---");
        telemetry.addData("Distance", "%.1f in", currentDistance);
        telemetry.addData("TX Error", "%.2f°", filteredTX);
        telemetry.addData("Yaw Angle", "%.2f° (%.3f)", currentYawAngle, currentYawServoPos);
        telemetry.addData("Shooter", "%.3f (%.0f tps)", currentShooterPower, currentShooterVelocity);
        telemetry.addLine("");

        telemetry.addLine("--- Manual Adjustments ---");
        telemetry.addData("Power Adj", "%+.3f (Y/A)", shooterPowerAdjustment);
        telemetry.addData("Angle Adj", "%+.3f (X/B)", yawAngleAdjustment);
        telemetry.addLine("");

        telemetry.addLine("--- Controls ---");
        telemetry.addLine("Y/A: Shooter Power ±");
        telemetry.addLine("X/B: Yaw Angle ±");
        telemetry.addLine("↑: Record SUCCESS");
        telemetry.addLine("↓: Record MISS");
        telemetry.addLine("→: Calculate");
        telemetry.addLine("←: Export CSV");

        telemetry.update();
    }
}