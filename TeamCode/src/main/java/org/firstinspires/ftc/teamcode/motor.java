package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class motor extends LinearOpMode {

    DcMotor motor;

    boolean motorOn = false;
    boolean lastAState = false;

    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    double power = 0.5;
    final double POWER_STEP = 0.05;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Toggle motor on/off with A
            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState) {
                motorOn = !motorOn;
            }
            lastAState = currentAState;

            // Increase power with D-pad up
            if (gamepad1.dpad_up && !lastDpadUp) {
                power += POWER_STEP;
            }

            // Decrease power with D-pad down
            if (gamepad1.dpad_down && !lastDpadDown) {
                power -= POWER_STEP;
            }

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Clamp power between 0 and 1
            power = Range.clip(power, 0.0, 1.0);

            motor.setPower(motorOn ? power : 0.0);

            telemetry.addData("Motor On", motorOn);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
