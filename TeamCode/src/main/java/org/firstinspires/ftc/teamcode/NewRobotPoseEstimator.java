package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * RobotPoseEstimator
 *
 * Fuses GoBilda Pinpoint odometry with Limelight 3A AprilTag field-pose using a
 * 3-state Extended Kalman Filter (EKF).
 *
 * State vector: [x (mm), y (mm), heading (radians)]
 *
 * HOW IT WORKS:
 *   - Pinpoint runs every loop (~50 Hz) as the PREDICTION step.
 *     It's fast and low-latency but drifts over time.
 *   - Limelight runs whenever it sees an AprilTag as the CORRECTION step.
 *     It's slower and noisier frame-to-frame but has no long-term drift.
 *   - The EKF blends both based on their relative trust (noise matrices).
 *
 * COORDINATE SYSTEM:
 *   Matches Limelight's field-space output (meters → converted to mm internally).
 *   Origin = wherever Limelight's field map defines (0,0).
 *   +X = right along field, +Y = forward along field, heading 0 = facing +X.
 *
 * TEST FIELD SETUP (4 AprilTags, 36h11 family, printed at 6.5 inch / 165mm):
 *   Place tags on walls of your test space at approximately:
 *     Tag ID 1: North wall, center,  facing South  → field pos (0,    2000) mm
 *     Tag ID 2: South wall, center,  facing North  → field pos (0,   -2000) mm
 *     Tag ID 3: East wall,  center,  facing West   → field pos (2000,    0) mm
 *     Tag ID 4: West wall,  center,  facing East   → field pos (-2000,   0) mm
 *   Mount tags at ~1 meter height (roughly camera height for best accuracy).
 *   Print the tag layout and enter it into Limelight's field map at limelight.local:5801.
 *
 * USAGE:
 *   1. Construct once in your OpMode.
 *   2. Call update() every loop iteration.
 *   3. Call getPose() to get the current best estimate.
 *   4. Call isInitialized() before trusting the pose.
 */
public class NewRobotPoseEstimator {

    // ─── Tuning constants ────────────────────────────────────────────────────

    /**
     * Process noise: how much we expect the state to change unpredictably
     * per loop from odometry error, wheel slip, etc.
     * Increase if the filter lags behind reality. Decrease to smooth more.
     */
    private static final double Q_XY      = 1.0;    // mm²  per loop
    private static final double Q_HEADING = 0.0005; // rad² per loop

    /**
     * Measurement noise: how much we trust Limelight's field pose output.
     * Increase to trust Limelight less (rely more on odometry).
     * Decrease to trust Limelight more.
     */
    private static final double R_XY      = 2500.0; // mm²  (~50mm std dev)
    private static final double R_HEADING = 0.04;   // rad² (~11° std dev)

    /**
     * Minimum number of tags Limelight must see for an update to be accepted.
     * 1 = accept single-tag poses (noisier). 2 = require two tags (more accurate).
     * For a small test field with 4 tags, 1 is fine.
     */
    private static final int MIN_TAGS_FOR_UPDATE = 1;

    /**
     * Maximum plausible jump in position per loop from Limelight (mm).
     * Readings that jump farther than this are rejected as outliers.
     */
    private static final double MAX_POSITION_JUMP_MM = 1500.0;

    /**
     * How long to wait for Limelight auto-init before requiring driver selection (ms).
     */
    private static final long AUTO_INIT_TIMEOUT_MS = 5000;

    // ─── EKF state ───────────────────────────────────────────────────────────

    // State: [x (mm), y (mm), heading (rad)]
    private double stateX       = 0.0;
    private double stateY       = 0.0;
    private double stateHeading = 0.0;

    // Covariance matrix P (3x3, stored as flat array row-major)
    // P[i][j] = p[i*3 + j]
    private double[] P = new double[9];

    // ─── Internal state ──────────────────────────────────────────────────────

    private final GoBildaPinpointDriver odo;
    private final Limelight3A           limelight;

    private boolean initialized       = false;
    private boolean limelightInitDone = false;
    private long    startTimeMs       = -1;

    // Last known Pinpoint pose, used to compute delta each loop
    private double lastOdoX       = 0.0;
    private double lastOdoY       = 0.0;
    private double lastOdoHeading = 0.0;
    private boolean firstOdoRead  = true;

    // ─── Constructor ─────────────────────────────────────────────────────────

    /**
     * @param odo       Already-initialized GoBildaPinpointDriver (call resetPosAndIMU before passing)
     * @param limelight Already-started Limelight3A (call pipelineSwitch + start before passing)
     */
    public NewRobotPoseEstimator(GoBildaPinpointDriver odo, Limelight3A limelight) {
        this.odo       = odo;
        this.limelight = limelight;

        // Start with high uncertainty so first good measurement dominates
        setCovariance(1e8, 1e8, 1e4);
    }

    // ─── Public API ──────────────────────────────────────────────────────────

    /**
     * Call every loop iteration. Runs the full EKF predict + update cycle.
     *
     * @param currentTimeMs System.currentTimeMillis() — used for auto-init timeout
     */
    public void update(long currentTimeMs) {
        if (startTimeMs < 0) startTimeMs = currentTimeMs;

        // Step 1: Try Limelight auto-init if not yet initialized
        if (!initialized) {
            tryAutoInit();
            // After timeout, caller should be offering driver selection
            return;
        }

        // Step 2: Predict from Pinpoint odometry delta
        odo.update();
        predictFromOdometry();

        // Step 3: Feed current heading to Limelight for MegaTag2
        limelight.updateRobotOrientation(Math.toDegrees(stateHeading));

        // Step 4: Correct from Limelight MT2 if a good pose is available
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            int tagCount = countVisibleTags(result);
            if (tagCount >= MIN_TAGS_FOR_UPDATE) {
                Pose3D botPose = result.getBotpose_MT2(); // MegaTag2
                if (botPose != null) {
                    correctFromLimelight(botPose);
                }
            }
        }
    }

    /**
     * Manually set the robot's starting pose (Option 2 fallback).
     * Call this if auto-init times out and the driver selects a position.
     */
    public void forceInitialize(double xMM, double yMM, double headingDeg) {
        stateX       = xMM;
        stateY       = yMM;
        stateHeading = Math.toRadians(headingDeg);

        // Moderate uncertainty — we know roughly where we are
        setCovariance(10000, 10000, 0.1);

        syncOdometryBaseline();
        initialized = true;
    }

    /** @return true once the estimator has an initial pose and is running */
    public boolean isInitialized() { return initialized; }

    /**
     * @return true if auto-init timed out and the driver should select a position.
     *         Check this each loop during init phase.
     */
    public boolean needsDriverSelection(long currentTimeMs) {
        return !initialized && startTimeMs > 0
                && (currentTimeMs - startTimeMs) > AUTO_INIT_TIMEOUT_MS;
    }

    /** @return current best-estimate X position in mm */
    public double getX() { return stateX; }

    /** @return current best-estimate Y position in mm */
    public double getY() { return stateY; }

    /** @return current best-estimate heading in degrees */
    public double getHeadingDegrees() { return Math.toDegrees(stateHeading); }

    /** @return current best-estimate heading in radians */
    public double getHeadingRadians() { return stateHeading; }

    /**
     * Convenience: get pose as a Pose2D in millimeters and degrees.
     */
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.MM, stateX, stateY,
                AngleUnit.DEGREES, Math.toDegrees(stateHeading));
    }

    /**
     * Diagnostic: get position uncertainty (standard deviation) in mm.
     * Lower = more confident. Watch this go down after Limelight corrections.
     */
    public double getPositionUncertaintyMM() {
        // sqrt of average of x and y diagonal covariance
        return Math.sqrt((P[0] + P[4]) / 2.0);
    }

    // ─── EKF Predict Step ────────────────────────────────────────────────────

    /**
     * Prediction step: propagate state forward using Pinpoint odometry delta.
     * This runs every loop regardless of Limelight visibility.
     */
    private void predictFromOdometry() {
        Pose2D pos = odo.getPosition();
        double odoX       = pos.getX(DistanceUnit.MM);
        double odoY       = pos.getY(DistanceUnit.MM);
        double odoHeading = pos.getHeading(AngleUnit.RADIANS);

        if (firstOdoRead) {
            lastOdoX       = odoX;
            lastOdoY       = odoY;
            lastOdoHeading = odoHeading;
            firstOdoRead   = false;
            return;
        }

        // Compute delta in odometry frame
        double dOdoX    = odoX - lastOdoX;
        double dOdoY    = odoY - lastOdoY;
        double dHeading = normalizeAngle(odoHeading - lastOdoHeading);

        lastOdoX       = odoX;
        lastOdoY       = odoY;
        lastOdoHeading = odoHeading;

        // State prediction (Pinpoint already outputs field-frame, no rotation needed)
        stateX += dOdoX * Math.cos(stateHeading) - dOdoY * Math.sin(stateHeading);
        stateY += dOdoX * Math.sin(stateHeading) + dOdoY * Math.cos(stateHeading);
        stateHeading  = normalizeAngle(stateHeading + dHeading);

        // Covariance prediction: P = F*P*F^T + Q
        // For this near-linear system F ≈ I (identity), so P = P + Q
        P[0] += Q_XY;
        P[4] += Q_XY;
        P[8] += Q_HEADING;
    }

    // ─── EKF Update Step ─────────────────────────────────────────────────────

    /**
     * Correction step: update state using Limelight field-pose measurement.
     * Uses standard Kalman gain calculation with outlier rejection.
     */
    private void correctFromLimelight(Pose3D botPose) {
        // Convert Limelight meters → mm
        double measX       = botPose.getPosition().x * 1000.0;
        double measY       = botPose.getPosition().y * 1000.0;
        double measHeading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Outlier rejection: ignore if pose jumps unrealistically far
        double dx = measX - stateX;
        double dy = measY - stateY;
        if (Math.sqrt(dx * dx + dy * dy) > MAX_POSITION_JUMP_MM) return;

        // Innovation (measurement residual)
        double innX = dx;
        double innY = dy;
        double innH = normalizeAngle(measHeading - stateHeading);

        // S = P + R  (innovation covariance, diagonal since H = I)
        double Sx = P[0] + R_XY;
        double Sy = P[4] + R_XY;
        double Sh = P[8] + R_HEADING;

        // Kalman gain K = P * H^T * S^-1  (simplifies to P[i] / S[i] on diagonal)
        double Kx = P[0] / Sx;
        double Ky = P[4] / Sy;
        double Kh = P[8] / Sh;

        // State update
        stateX       += Kx * innX;
        stateY       += Ky * innY;
        stateHeading  = normalizeAngle(stateHeading + Kh * innH);

        // Covariance update: P = (I - K*H) * P
        P[0] *= (1.0 - Kx);
        P[4] *= (1.0 - Ky);
        P[8] *= (1.0 - Kh);
    }

    // ─── Auto-Init ───────────────────────────────────────────────────────────

    /**
     * Attempt to initialize pose from Limelight MT2 on first valid field-pose reading.
     * Requires seeing at least MIN_TAGS_FOR_UPDATE tags.
     */
    private void tryAutoInit() {
        if (limelightInitDone) return;

        odo.update();

        // Feed heading to Limelight for MegaTag2 even during init
        limelight.updateRobotOrientation(Math.toDegrees(stateHeading));

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        int tagCount = countVisibleTags(result);
        if (tagCount < MIN_TAGS_FOR_UPDATE) return;

        Pose3D botPose = result.getBotpose_MT2(); // MegaTag2
        if (botPose == null) return;

        double x       = botPose.getPosition().x * 1000.0; // m → mm
        double y       = botPose.getPosition().y * 1000.0;
        double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

        stateX       = x;
        stateY       = y;
        stateHeading = heading;

        // After Limelight init, trust is moderate — will tighten fast
        setCovariance(5000, 5000, 0.05);

        syncOdometryBaseline();
        limelightInitDone = true;
        initialized       = true;
    }

    // ─── Helpers ─────────────────────────────────────────────────────────────

    /** Sync the last-known odometry values to current, so first delta is zero. */
    private void syncOdometryBaseline() {
        Pose2D pos     = odo.getPosition();
        lastOdoX       = pos.getX(DistanceUnit.MM);
        lastOdoY       = pos.getY(DistanceUnit.MM);
        lastOdoHeading = pos.getHeading(AngleUnit.RADIANS);
        firstOdoRead   = false;
    }

    /** Set diagonal covariance values (off-diagonals = 0). */
    private void setCovariance(double px, double py, double ph) {
        P = new double[9]; // zero all
        P[0] = px;
        P[4] = py;
        P[8] = ph;
    }

    /** Normalize angle to [-π, π]. */
    private double normalizeAngle(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    /** Count visible AprilTags in the latest result. */
    private int countVisibleTags(LLResult result) {
        if (result.getFiducialResults() == null) return 0;
        return result.getFiducialResults().size();
    }
}