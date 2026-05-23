package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * StartingPositionSelector
 *
 * Shown on the Driver Station when Limelight auto-init fails (no tags visible
 * in the first 5 seconds). Driver uses gamepad buttons to pick a known starting
 * position, which is then used to seed the EKF.
 *
 * TEST FIELD POSITIONS (match your tag layout):
 *   These assume the 4-tag layout described in RobotPoseEstimator.java.
 *   Adjust X/Y values (in mm) if your field is a different size.
 *
 *   A = South wall start, facing North  (0, -1800 mm, 90°)
 *   B = North wall start, facing South  (0,  1800 mm, 270°)
 *   X = West wall start,  facing East   (-1800, 0 mm, 0°)
 *   Y = East wall start,  facing West   (1800,  0 mm, 180°)
 *
 * USAGE:
 *   StartingPositionSelector selector = new StartingPositionSelector(telemetry);
 *   // In your loop (before waitForStart or after auto-init timeout):
 *   selector.update(gamepad1);
 *   if (selector.isSelected()) {
 *       poseEstimator.forceInitialize(
 *           selector.getX(), selector.getY(), selector.getHeadingDeg());
 *   }
 */
public class StartingPositionSelector {

    public static class StartingPosition {
        public final String name;
        public final double xMM;
        public final double yMM;
        public final double headingDeg;

        public StartingPosition(String name, double xMM, double yMM, double headingDeg) {
            this.name       = name;
            this.xMM        = xMM;
            this.yMM        = yMM;
            this.headingDeg = headingDeg;
        }
    }

    // ─── Preset positions — edit these to match your test field ──────────────
    private static final StartingPosition[] POSITIONS = {
            new StartingPosition("South Wall → Facing North",   0,    -1800,  90),
            new StartingPosition("North Wall → Facing South",   0,     1800, 270),
            new StartingPosition("West Wall  → Facing East",  -1800,     0,    0),
            new StartingPosition("East Wall  → Facing West",   1800,     0,  180),
    };

    private final Telemetry telemetry;
    private StartingPosition selected = null;

    // Debounce — prevent one button press registering twice
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;

    public StartingPositionSelector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Call every loop while waiting for driver selection.
     * Draws the menu and detects button presses.
     */
    public void update(Gamepad gamepad) {
        if (selected != null) return; // Already chosen

        // Detect rising edges only (button just pressed, not held)
        boolean pressA = gamepad.a && !lastA;
        boolean pressB = gamepad.b && !lastB;
        boolean pressX = gamepad.x && !lastX;
        boolean pressY = gamepad.y && !lastY;

        lastA = gamepad.a;
        lastB = gamepad.b;
        lastX = gamepad.x;
        lastY = gamepad.y;

        if (pressA) selected = POSITIONS[0];
        if (pressB) selected = POSITIONS[1];
        if (pressX) selected = POSITIONS[2];
        if (pressY) selected = POSITIONS[3];

        // Draw menu on Driver Station
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("  AUTO-INIT TIMED OUT");
        telemetry.addLine("  Select starting position:");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addData("  [A]", POSITIONS[0].name);
        telemetry.addData("  [B]", POSITIONS[1].name);
        telemetry.addData("  [X]", POSITIONS[2].name);
        telemetry.addData("  [Y]", POSITIONS[3].name);
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.update();
    }

    /** @return true once driver has pressed a button */
    public boolean isSelected() { return selected != null; }

    public double getX()          { return selected != null ? selected.xMM        : 0; }
    public double getY()          { return selected != null ? selected.yMM        : 0; }
    public double getHeadingDeg() { return selected != null ? selected.headingDeg : 0; }
    public String getName()       { return selected != null ? selected.name       : "None"; }
}