package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "odometry debug", group = "Debug")
public class OdometryDebug extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(0, 6, DistanceUnit.MM);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        telemetry.addLine("hold still then start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            Pose2D pos = odo.getPosition();
            double x = pos.getX(DistanceUnit.MM);
            double y = pos.getY(DistanceUnit.MM);
            double h = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addLine("push forward and x should increase");
            telemetry.addData("X (mm)", "%.1f", x);
            telemetry.addLine("push right and y should increase");
            telemetry.addData("Y (mm)", "%.1f", y);
            telemetry.addLine("spin counterclockwise and heading should increase");
            telemetry.addData("Heading (degrees)", "%.1f", h);
            telemetry.addLine("");
            telemetry.addLine("Press A to zero");
            telemetry.update();

            if (gamepad1.a) {
                odo.resetPosAndIMU();
                sleep(300);
            }
        }
    }
}