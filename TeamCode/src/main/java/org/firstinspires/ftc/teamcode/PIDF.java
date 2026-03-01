package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF extends OpMode {
    public DcMotorEx shooter;
    public double highVelocity = 790;
    public double lowVelocity = 650;
    double curTargetVelocity = highVelocity;
    double F =0;
    double P = 0;
    double[] stepsizes= {0.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex=1;
    @Override
    public void init(){
shooter = hardwareMap.get(DcMotorEx.class, "shooter");
shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
    shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    telemetry.addLine("Init complete");
    }
    @Override
    public void loop(){
if (gamepad1.yWasPressed()){
    if (curTargetVelocity==highVelocity){
        curTargetVelocity=lowVelocity;
    }else{curTargetVelocity = highVelocity;}
}
if (gamepad1.bWasPressed()){
    stepIndex = (stepIndex+1) % stepsizes.length;
}
if (gamepad1.dpadLeftWasPressed()){
    F+=stepsizes[stepIndex];

}
if (gamepad1.dpadRightWasPressed()){
    F-=stepsizes[stepIndex];
}
if(gamepad1.dpadUpWasPressed()){
    P+=stepsizes[stepIndex];
}
if(gamepad1.dpadDownWasPressed()){
    P-=stepsizes[stepIndex];
}
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
shooter.setVelocity(curTargetVelocity);

double curvelocity = shooter.getVelocity();
double error = curTargetVelocity-curvelocity;

telemetry.addData("Target Velocity", curTargetVelocity);
telemetry.addData("Current Velocity", "%.2f", curvelocity);
telemetry.addData("Error", "%.2f", error);
telemetry.addLine("------------------------------------------------");
telemetry.addData("Tuning P", "%.4f (dpad Up and down)", P);
telemetry.addData("Tuning F", "%.4f (left Up and right)", F);
telemetry.addData("Step Size", "%.4f (B botton)", stepsizes[stepIndex]);
    }
}
