package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name="TheRobot_AutoAim_v2", group="Competition")
public class k extends LinearOpMode {

    // --- Drive ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Turret ---
    private DcMotor turretPitch;  // Motor for up/down (reversed)
    private Servo turretYaw;      // Servo for left/right rotation

    // --- Vision ---
    private Limelight3A limelight;

    // --- Config ---
    private static final int TARGET_TAG_ID = 24; // Red Bucket (change if needed)

    // Remembered target position (in inches, field coordinates)
    private double lastKnownTargetX = 0;
    private double lastKnownTargetY = 0;
    private boolean hasSeenTag = false;

    // Turret Motor (Pitch - up/down angle)
    private static final double PITCH_POWER_SCALE = 0.5;

    // Turret Servo (Yaw - left/right, but NOT auto-controlled for aiming)
    private static final double YAW_MIN = 0.25;
    private static final double YAW_MAX = 0.5828;
    private static final double YAW_CENTER = (YAW_MIN + YAW_MAX) / 2;

    @Override
    public void runOpMode() {
        initHardware();
        limelight.start();

        telemetry.addData("Status", "Initialized - Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            driveControl();

            LLResult result = limelight.getLatestResult();
            boolean tagVisible = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && fiducials.size() > 0) {
                    telemetry.addData("Fiducials Found", fiducials.size());

                    for (LLResultTypes.FiducialResult f : fiducials) {
                        telemetry.addData("Fiducial ID", f.getFiducialId());

                        if ((int) f.getFiducialId() == TARGET_TAG_ID) {
                            tagVisible = true;

                            // Get tag pose in robot space
                            Pose3D tagPoseRobot = f.getTargetPoseRobotSpace();

                            // Get robot's field position
                            Pose3D botpose = result.getBotpose();

                            telemetry.addData("Tag Pose Null?", tagPoseRobot == null);
                            telemetry.addData("Botpose Null?", botpose == null);

                            if (tagPoseRobot != null && botpose != null) {
                                Position robotPos = botpose.getPosition();
                                YawPitchRollAngles robotOrientation = botpose.getOrientation();

                                // Convert meters to inches
                                double robotX = robotPos.x * 39.3701;
                                double robotY = robotPos.y * 39.3701;
                                double robotYaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

                                // Get tag position relative to robot (in meters, convert to inches)
                                Position tagPosRobot = tagPoseRobot.getPosition();
                                double tagX_robot = tagPosRobot.x * 39.3701;  // forward
                                double tagY_robot = tagPosRobot.y * 39.3701;  // left

                                telemetry.addData("Robot X (field)", "%.1f", robotX);
                                telemetry.addData("Robot Y (field)", "%.1f", robotY);
                                telemetry.addData("Robot Yaw", "%.1f", robotYaw);

                                // Transform tag position from robot space to field space
                                double robotYawRad = Math.toRadians(robotYaw);
                                double cosYaw = Math.cos(robotYawRad);
                                double sinYaw = Math.sin(robotYawRad);

                                // Field coordinates of the tag
                                double newTargetX = robotX + tagX_robot * cosYaw - tagY_robot * sinYaw;
                                double newTargetY = robotY + tagX_robot * sinYaw + tagY_robot * cosYaw;

                                // Only update if position changed significantly (more than 2 inches)
                                // This prevents jitter from small vision errors
                                if (!hasSeenTag ||
                                        Math.abs(newTargetX - lastKnownTargetX) > 2 ||
                                        Math.abs(newTargetY - lastKnownTargetY) > 2) {

                                    lastKnownTargetX = newTargetX;
                                    lastKnownTargetY = newTargetY;
                                    hasSeenTag = true;

                                    telemetry.addData("UPDATED TARGET!", "");
                                }

                                telemetry.addData("Tag Robot X", "%.1f", tagX_robot);
                                telemetry.addData("Tag Robot Y", "%.1f", tagY_robot);
                            }
                            break;
                        }
                    }
                } else {
                    telemetry.addData("Fiducials", "None detected");
                }
            } else {
                telemetry.addData("LL Result", "Invalid or null");
            }

            // Aiming logic (works even if tag not visible)
            if (hasSeenTag) {
                aimAtRememberedTag();
            } else {
                // No target acquired yet - stop the pitch motor
                turretPitch.setPower(0);
            }

            telemetry.addData("=== STATUS ===", "");
            telemetry.addData("Tag Visible NOW", tagVisible);
            telemetry.addData("Has EVER Seen Tag", hasSeenTag);
            telemetry.addData("--- Remembered Target ---", "");
            telemetry.addData("Target Field X", "%.1f in", lastKnownTargetX);
            telemetry.addData("Target Field Y", "%.1f in", lastKnownTargetY);
            telemetry.update();
        }
    }

    private void aimAtRememberedTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Aim Status", "No valid result");
            turretPitch.setPower(0);
            return;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            telemetry.addData("Aim Status", "No botpose");
            turretPitch.setPower(0);
            return;
        }

        Position robotPos = botpose.getPosition();
        YawPitchRollAngles robotOrientation = botpose.getOrientation();

        double robotX = robotPos.x * 39.3701;
        double robotY = robotPos.y * 39.3701;
        double robotYaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        double robotYawRad = Math.toRadians(robotYaw);

        // Compute vector to target in field frame
        double dx = lastKnownTargetX - robotX;
        double dy = lastKnownTargetY - robotY;
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);

        // Angle to target in field frame (radians)
        double targetAngleField = Math.atan2(dy, dx);

        // Convert to robot frame (relative to robot's current heading)
        // This is the horizontal angle the turret needs to rotate
        double horizontalAngleRobot = targetAngleField - robotYawRad;

        // Normalize to [-PI, PI]
        while (horizontalAngleRobot < -Math.PI) horizontalAngleRobot += 2 * Math.PI;
        while (horizontalAngleRobot > Math.PI) horizontalAngleRobot -= 2 * Math.PI;

        // Use MOTOR (turretPitch) to rotate left/right to aim at target
        // Convert angle to motor power (proportional control)
        // Negative angle = target is to the left, positive = target is to the right
        double motorPower = horizontalAngleRobot / Math.PI * PITCH_POWER_SCALE;

        // Add deadband to stop when close enough (within ~3 degrees)
        if (Math.abs(horizontalAngleRobot) < Math.toRadians(3)) {
            motorPower = 0;
        }

        // Clamp power to valid range
        motorPower = Math.max(-PITCH_POWER_SCALE, Math.min(PITCH_POWER_SCALE, motorPower));

        // Apply power to motor
        turretPitch.setPower(motorPower);

        telemetry.addData("--- Current Robot Pos ---", "");
        telemetry.addData("Robot Field X", "%.1f in", robotX);
        telemetry.addData("Robot Field Y", "%.1f in", robotY);
        telemetry.addData("Robot Yaw", "%.1f°", robotYaw);
        telemetry.addData("--- Aiming Data ---", "");
        telemetry.addData("DX", "%.1f in", dx);
        telemetry.addData("DY", "%.1f in", dy);
        telemetry.addData("Distance", "%.1f in", distanceToTarget);
        telemetry.addData("Horizontal Angle", "%.1f°", Math.toDegrees(horizontalAngleRobot));
        telemetry.addData("Raw Motor Power", "%.3f", motorPower);
        telemetry.addData("Motor Direction", motorPower > 0 ? "RIGHT" : motorPower < 0 ? "LEFT" : "STOPPED");
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        turretPitch = hardwareMap.get(DcMotor.class, "turretPitch");
        turretPitch.setDirection(DcMotor.Direction.REVERSE);
        turretPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretYaw = hardwareMap.get(Servo.class, "turretYaw");
        turretYaw.setPosition(YAW_CENTER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    private void driveControl() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}