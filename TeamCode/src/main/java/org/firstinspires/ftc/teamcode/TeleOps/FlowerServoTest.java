package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// FlowerServoTest.java — test OpMode for the two flower servos.
// Both servos start at 0 and always get the SAME position.
//
// CONTROLS (gamepad 1)
//   dpad up   (hold) = move both servos up   (position increases)
//   dpad down (hold) = move both servos down (position decreases)
//   dpad right (press) = nudge up by FINE_STEP     dpad left (press) = nudge down by FINE_STEP
//
// HARDWARE: servos "flowerleft" and "flowerright" (change the names below to match the Robot Configuration).
// If the two servos move in opposite directions, flip REVERSE_RIGHT / REVERSE_LEFT.
// Read the final position from telemetry and put it in your real code.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FlowerServoTest", group = "Test")
@Disabled
public class FlowerServoTest extends OpMode {

    private static final String LEFT_NAME = "linkageright";
    private static final String RIGHT_NAME = "linkageleft";
    private static final boolean REVERSE_LEFT = false;
    private static final boolean REVERSE_RIGHT = true; // mirrored servos usually need one reversed

    private static final double START_POSITION = 0.7;
    private static final double SPEED_PER_SEC = 0.1; // position change per second while dpad up/down is held
    private static final double FINE_STEP = 0.01;    // dpad left/right press

    private Servo left, right;
    private double position = START_POSITION;
    private boolean lastLeft, lastRight;
    private final ElapsedTime dtTimer = new ElapsedTime();

    private void apply() {
        position = Math.max(0.0, Math.min(1.0, position));
        left.setPosition(position);
        right.setPosition(position);
    }

    @Override
    public void init() {
        left = hardwareMap.get(Servo.class, LEFT_NAME);
        right = hardwareMap.get(Servo.class, RIGHT_NAME);
        left.setDirection(REVERSE_LEFT ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        right.setDirection(REVERSE_RIGHT ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        position = START_POSITION;
        apply();
        telemetry.addLine("Both servos set to " + START_POSITION + ". Press PLAY, then dpad up / down.");
        telemetry.update();
    }

    @Override
    public void start() {
        dtTimer.reset();
    }

    @Override
    public void loop() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        if (gamepad1.dpad_up) position += SPEED_PER_SEC * dt;
        if (gamepad1.dpad_down) position -= SPEED_PER_SEC * dt;

        boolean dr = gamepad1.dpad_right, dl = gamepad1.dpad_left;
        if (dr && !lastRight) position += FINE_STEP;
        if (dl && !lastLeft) position -= FINE_STEP;
        lastRight = dr;
        lastLeft = dl;

        apply();

        telemetry.addData("Position (both servos)", "%.3f", position);
        telemetry.addLine("dpad up/down = move (hold) | dpad left/right = fine nudge");
        telemetry.update();
    }
}
