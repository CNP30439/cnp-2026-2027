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
    public DcMotorEx rs;
 
    public double highVelocity =4300;//Max RPM 6943
    public double lowVelocity =2600;
 
    double curTargetVelocity = lowVelocity;
 
    double F = 14.366;
 
    double P = 1.7000;
 
    // Full decade progression so you can walk gains down smoothly instead of
    // jumping 100x from 0.1 straight to 0.001 (0.01 was missing before).
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
 
    int stepIndex = 1;
 
 
    @Override
    public void init() {
        ls = hardwareMap.get(DcMotorEx.class,"ls");
        rs = hardwareMap.get(DcMotorEx.class,"rs");
        ls.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);
 
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        ls.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        rs.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
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
        rs.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
 
        ls.setVelocity(curTargetVelocity);
        rs.setVelocity(curTargetVelocity);
 
        double curVelocity = ls.getVelocity();
        double curVelocity2 = -rs.getVelocity();
        double error = curTargetVelocity - curVelocity;
        double error2 = curTargetVelocity - curVelocity2;
 
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Current Velocity2", "%.2f", curVelocity2);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Error2", "%.2f", error2);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
 
    }
 
 
 
 
 
 
 
}
 