package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
 steps:
 *   1. mount Pinpoint sticker side up on chassis.
 *   2. Mount X pod tracking forward, Y pod tracking sideways (left/right).
 *   3. Name the device odo in Robot Configuration.
 *   4. Set your pod type and offsets in the CONFIGURATION section below.
 *
 * gamepad controls:
 *   A = reset position and recalibrate IMU (use when robot is still)
 *   B = recalibrate IMU only
 *   X = Reverse X encoder direction
 *   Y = Reverse Y encoder direction
 */

@TeleOp(name = "Pinpoint Full Setup", group = "Testing")
public class PinpointFullSetup extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    // -----------------------------------------------------------------------
    // config
    // -----------------------------------------------------------------------

    // Measure these on robot in mm:
    // X offset: how far left of center the X (forward) pod is. Left = positive.
    // Y offset: how far forward of center the Y (strafe) pod is. Forward = positive.
    static final double X_OFFSET = -25.4;
    static final double Y_OFFSET = 0;

    static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // -----------------------------------------------------------------------

    // Track encoder directions so we can toggle them with gamepad
    private GoBildaPinpointDriver.EncoderDirection xDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection yDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Apply configuration
        odo.setEncoderResolution(POD_TYPE);
        odo.setOffsets(X_OFFSET, Y_OFFSET, DistanceUnit.MM);
        odo.setEncoderDirections(xDir, yDir);

        // Reset position and IMU — robot MUST be still
        odo.resetPosAndIMU();

        telemetry.addLine("Pinpoint initialized.");
        telemetry.addLine("Press START when ready.");
        telemetry.addData("Device Status", odo.getDeviceStatus());
        telemetry.addData("X Offset (mm)", X_OFFSET);
        telemetry.addData("Y Offset (mm)", Y_OFFSET);
        telemetry.addData("Pod Type", POD_TYPE);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            // --- Gamepad Controls ---

            // A = reset position + recalibrate IMU (robot must be still)
            if (gamepad1.a) {
                odo.resetPosAndIMU();
                telemetry.addLine("Reset position and IMU!");
            }

            // B = recalibrate IMU only (keeps position)
            if (gamepad1.b) {
                odo.recalibrateIMU();
                telemetry.addLine("Recalibrated IMU only!");
            }

            // X = toggle X encoder direction
            if (gamepad1.x) {
                xDir = (xDir == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD;
                odo.setEncoderDirections(xDir, yDir);
                sleep(300); // debounce
            }

            // Y = toggle Y encoder direction
            if (gamepad1.y) {
                yDir = (yDir == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD;
                odo.setEncoderDirections(xDir, yDir);
                sleep(300); // debounce
            }

            // --- Read Data ---
            Pose2D pos = odo.getPosition();
            double xPos    = pos.getX(DistanceUnit.MM);
            double yPos    = pos.getY(DistanceUnit.MM);
            double heading = pos.getHeading(AngleUnit.DEGREES);

            int rawX = odo.getEncoderX();
            int rawY = odo.getEncoderY();

            GoBildaPinpointDriver.DeviceStatus status = odo.getDeviceStatus();

            // --- Telemetry ---

            telemetry.addLine("========== DEVICE STATUS ==========");
            telemetry.addData("Status", status);
            telemetry.addData("Pinpoint Frequency (Hz)", String.format(Locale.US, "%.1f", odo.getFrequency()));

            telemetry.addLine("");
            telemetry.addLine("========== POSITION ==========");
            telemetry.addData("X Position (mm)", String.format(Locale.US, "%.2f", xPos));
            telemetry.addData("Y Position (mm)", String.format(Locale.US, "%.2f", yPos));
            telemetry.addData("Heading (degrees)", String.format(Locale.US, "%.2f", heading));

            telemetry.addLine("");
            telemetry.addLine("========== RAW ENCODERS ==========");
            telemetry.addData("Raw X Encoder (ticks)", rawX);
            telemetry.addData("Raw Y Encoder (ticks)", rawY);

            telemetry.addLine("");
            telemetry.addLine("========== ENCODER DIRECTIONS ==========");
            telemetry.addData("X Direction", xDir);
            telemetry.addData("Y Direction", yDir);

            telemetry.addLine("");
            telemetry.addLine("========== SETUP CHECKLIST ==========");
            telemetry.addData("Step 5", status == GoBildaPinpointDriver.DeviceStatus.READY
                    ? "PASS - Device is READY (LED green)" : "FAIL - Check pods/connections");
            telemetry.addData("Step 7", "Move FORWARD → X should increase");
            telemetry.addData("Step 8", "Strafe LEFT  → Y should increase");
            telemetry.addData("Step 10","Spin in place → X and Y should stay near 0");
            telemetry.addData("Step 11","Rotate CCW full turn → Heading should reach ~360°");

            telemetry.addLine("");
            telemetry.addLine("========== GAMEPAD ==========");
            telemetry.addData("A", "Reset position + IMU (keep still!)");
            telemetry.addData("B", "Recalibrate IMU only");
            telemetry.addData("X", "Toggle X encoder direction");
            telemetry.addData("Y", "Toggle Y encoder direction");

            telemetry.update();
        }
    }
}