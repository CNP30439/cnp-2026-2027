package org.firstinspires.ftc.teamcode.TurretCodes;
 
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
 
@TeleOp (name = "FlywheelTuner", group = "TeleOp")
public class FlywheelTuner extends OpMode {
    public DcMotorEx ls;


    double F = 12.84;   // or your measured value
    double P = 0;

    double highVelocity = 4300 / 1.5 * 28.0 / 60.0;  // ~1338 ticks/s  6814 rpm max
    double lowVelocity  = 2600 / 1.5 * 28.0 / 60.0;  // ~809 ticks/s
 
    double curTargetVelocity = lowVelocity;
 

 
    // Full decade progression so you can walk gains down smoothly instead of
    // jumping 100x from 0.1 straight to 0.001 (0.01 was missing before).
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
 
    int stepIndex = 1;
 
 
    @Override
    public void init() {
        ls = hardwareMap.get(DcMotorEx.class,"ls");

        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ls.setDirection(DcMotorSimple.Direction.FORWARD);

 
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        ls.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        telemetry.addLine("Init complete");
 
 
 
 
    }
 
    @Override
    public void loop(){
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
 
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
 
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
 
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
 
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        ls.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        ls.setVelocity(curTargetVelocity);

 
        double curVelocity = ls.getVelocity();

        double error = curTargetVelocity - curVelocity;

 
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);

        telemetry.addData("Error", "%.2f", error);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
 
    }
 
 
 
 
 
 
 
}
 