package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// FlowerServoTest.java — test OpMode for the two flower servos.
// Both servos always get the SAME position. Starts at FLOWER_DOWN (0.0).
//
// CONTROLS (gamepad 1) — each press moves once
//   dpad up    = up   by STEP
//   dpad down  = down by STEP
//   dpad right = up   by FINE_STEP
//   dpad left  = down by FINE_STEP
//
// HARDWARE: "linkageright" (FORWARD) and "linkageleft" (REVERSED).
// Read the position from telemetry and put it in your real code.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flower Servo Test", group = "Test")
public class FlowerServoTest extends OpMode {

    private static final String FLOWER_A_NAME = "linkageright"; // FORWARD
    private static final String FLOWER_B_NAME = "linkageleft";  // REVERSED
    private static final double FLOWER_DOWN = 0.4;
    private static final double FLOWER_UP = 0.4;

    private static final double STEP = 0.1;      // per dpad up/down press
    private static final double FINE_STEP = 0.01; // per dpad right/left press

    private Servo flowerA;
    private Servo flowerB;
    private double position = FLOWER_DOWN;
    private boolean lastUp, lastDown, lastRight, lastLeft;

    private void apply() {
        position = Math.max(0.0, Math.min(1.0, position));
        flowerA.setPosition(position);
        flowerB.setPosition(position);
    }

    @Override
    public void init() {
        flowerA = hardwareMap.get(Servo.class, FLOWER_A_NAME);
        flowerB = hardwareMap.get(Servo.class, FLOWER_B_NAME);

        flowerA.setDirection(Servo.Direction.FORWARD);
        flowerB.setDirection(Servo.Direction.REVERSE);

        position = FLOWER_DOWN;
        apply();
        telemetry.addData("Start position", "%.3f", position);
        telemetry.addLine("Press PLAY, then use the dpad.");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean right = gamepad1.dpad_right, left = gamepad1.dpad_left;

        if (up && !lastUp)       position += STEP;
        if (down && !lastDown)   position -= STEP;
        if (right && !lastRight) position += FINE_STEP;
        if (left && !lastLeft)   position -= FINE_STEP;

        lastUp = up;
        lastDown = down;
        lastRight = right;
        lastLeft = left;

        apply();

        telemetry.addData("Position (both servos)", "%.3f", position);
        telemetry.addData("FLOWER_DOWN / FLOWER_UP", "%.2f / %.2f", FLOWER_DOWN, FLOWER_UP);
        telemetry.addLine("dpad up/down = ±0.05 | dpad right/left = ±0.01");
        telemetry.update();
    }
}