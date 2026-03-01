package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Simplified LimelightHelpers for FTC
 * Provides easy access to Limelight 3A data
 */
public class LimelightHelpers {

    private static Limelight3A limelight = null;
    private static LLResult lastResult = null;

    /**
     * Initialize the Limelight hardware
     * Call this once in your init() method
     */
    public static void initLimelight(Limelight3A ll) {
        limelight = ll;
        limelight.pipelineSwitch(0); // Default pipeline
        limelight.start(); // Start polling for data
    }

    /**
     * Update the latest result from Limelight
     * Call this in your loop before reading values
     */
    private static void updateResult() {
        if (limelight != null) {
            lastResult = limelight.getLatestResult();
        }
    }

    /**
     * Get horizontal offset to target (TX)
     * @param limelightName - not used in FTC, kept for compatibility
     * @return horizontal offset in degrees (-29.8 to +29.8)
     */
    public static double getTX(String limelightName) {
        updateResult();
        if (lastResult != null && lastResult.isValid()) {
            return lastResult.getTx();
        }
        return 0.0;
    }

    /**
     * Get vertical offset to target (TY)
     * @param limelightName - not used in FTC, kept for compatibility
     * @return vertical offset in degrees
     */
    public static double getTY(String limelightName) {
        updateResult();
        if (lastResult != null && lastResult.isValid()) {
            return lastResult.getTy();
        }
        return 0.0;
    }

    /**
     * Check if valid target is detected
     * @param limelightName - not used in FTC, kept for compatibility
     * @return true if valid target exists
     */
    public static boolean getTV(String limelightName) {
        updateResult();
        if (lastResult != null) {
            return lastResult.isValid();
        }
        return false;
    }

    /**
     * Get robot pose from Limelight (WPI Blue alliance coordinates)
     * @param limelightName - not used in FTC, kept for compatibility
     * @return Pose3D object with x, y, z, rotation (returns null if no valid target)
     */
    public static Pose3D getBotPose_WpiBlue(String limelightName) {
        updateResult();
        if (lastResult != null && lastResult.isValid()) {
            return lastResult.getBotpose();
        }
        return null;
    }

    /**
     * Get target area (percentage of image)
     * @param limelightName - not used in FTC, kept for compatibility
     * @return target area 0-100
     */
    public static double getTA(String limelightName) {
        updateResult();
        if (lastResult != null && lastResult.isValid()) {
            return lastResult.getTa();
        }
        return 0.0;
    }

    /**
     * Set LED mode
     * @param limelightName - not used in FTC
     * @param mode - 0=pipeline, 1=off, 2=blink, 3=on
     */
    public static void setLEDMode(String limelightName, int mode) {
        // FTC Limelight API doesn't expose LED control the same way
        // This is a placeholder for compatibility
    }

    /**
     * Set pipeline index
     * @param limelightName - not used in FTC
     * @param pipeline - pipeline index (0-9)
     */
    public static void setPipeline(String limelightName, int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }
}