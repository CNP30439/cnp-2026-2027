package org.firstinspires.ftc.teamcode.TeleOps;

// TurretPowerTest — find the smallest power that moves the turret (breakaway power).
// Gamepad 1:
//   dpad up / down    = power +/- 0.01
//   dpad right / left = power +/- 0.05
//   hold A            = run turret at +power
//   hold B            = run turret at -power
//   release           = power 0 (BRAKE)
// Start low, hold A, raise power until "MOVING" shows. Do the same with B for the other side.

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Power Test", group = "Test")
public class TurretPowerTest extends OpMode {

    private DcMotorEx turret;
    private double power = 0.05;
    private boolean lastUp, lastDown, lastLeft, lastRight, lastRunning;
    private int startTicks = 0;
    private double maxPlus = 0, maxMinus = 0; // smallest power seen moving each way (0 = not yet)

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwareMap.get(Servo.class, "homeservo").setPosition(0.0); // sensor arm out of the way
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp) power += 0.01;
        if (gamepad1.dpad_down && !lastDown) power -= 0.01;
        if (gamepad1.dpad_right && !lastRight) power += 0.05;
        if (gamepad1.dpad_left && !lastLeft) power -= 0.05;
        power = Math.max(0, Math.min(1, power));
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

        double out = gamepad1.a ? power : gamepad1.b ? -power : 0;
        boolean running = out != 0;
        if (running && !lastRunning) startTicks = turret.getCurrentPosition();
        lastRunning = running;
        turret.setPower(out);

        int moved = running ? turret.getCurrentPosition() - startTicks : 0;
        double vel = turret.getVelocity();
        boolean moving = running && Math.abs(vel) > 20; // ticks/s

        if (moving && out > 0 && (maxPlus == 0 || power < maxPlus)) maxPlus = power;
        if (moving && out < 0 && (maxMinus == 0 || power < maxMinus)) maxMinus = power;

        telemetry.addData("Power set", "%.2f", power);
        telemetry.addData("Sending", "%.2f %s", out, gamepad1.a ? "(A = +)" : gamepad1.b ? "(B = -)" : "");
        telemetry.addData("Status", moving ? "MOVING" : (running ? "stuck" : "off"));
        telemetry.addData("Velocity", "%.0f ticks/s", vel);
        telemetry.addData("Ticks moved this hold", "%d", moved);
        telemetry.addData("Smallest moving + (A)", maxPlus == 0 ? "-" : String.format("%.2f", maxPlus));
        telemetry.addData("Smallest moving - (B)", maxMinus == 0 ? "-" : String.format("%.2f", maxMinus));
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }
}
