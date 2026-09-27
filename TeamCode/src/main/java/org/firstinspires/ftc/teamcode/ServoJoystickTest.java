package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  ServoJoystickTest.java  —  Bench-test OpMode: drive ONE servo's position with
//                             a joystick or dpad. Useful for finding min/max/preset
//                             positions on a mechanism before wiring it into a
//                             real TeleOp.
//
//  HARDWARE
//    1x servo — "kicker"   (rename to whatever you're testing, e.g. "hood")
//
//  CONTROLS
//    Gamepad 1
//      left stick Y   — moves the servo position up/down (rate-based, not
//                        absolute — holding the stick keeps moving it)
//      dpad up        — nudge position by +0.1  (one step per press)
//      dpad down      — nudge position by -0.1  (one step per press)
//      dpad right     — nudge position by +0.01 (one step per press)
//      dpad left      — nudge position by -0.01 (one step per press)
//      A              — snap servo to position 0.0
//      B              — snap servo to position 1.0
//      X              — snap servo to position 0.5 (center)
//
//  WHY RATE-BASED INSTEAD OF DIRECT STICK -> POSITION
//    Servos only have a 0.0–1.0 range, and most sticks are noisy near center,
//    so mapping stick position directly to servo position makes it twitchy
//    and hard to land on an exact spot. Instead the stick controls a SPEED:
//    push and hold to sweep the servo, ease off near your target, let go to
//    stop exactly there. Much easier for finding a precise position by feel.
//
//  WHY THE DPAD IS EDGE-TRIGGERED
//    loop() runs ~50 times/sec, so if held-down dpad input added 0.1 every
//    tick it would blow past any target instantly. Instead each button press
//    is detected once (on the rising edge) and applies a single fixed nudge,
//    so tapping dpad up/down/left/right gives repeatable, precise steps.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoJoystickTest", group = "Test")
public class ServoJoystickTest extends OpMode {

    private Servo servo;

    // Current commanded position, tracked in software since Servo has no
    // reliable getPosition() readback on most hubs.
    private double position = 0.5;

    // Stick values below this are treated as zero, so a resting stick doesn't
    // slowly drift the servo from residual noise.
    private static final double STICK_DEADZONE = 0.05;

    // How far the position moves per loop tick at full stick deflection.
    // Loop runs roughly every 20ms, so 0.01 here is a full sweep (0 to 1) in
    // about 2 seconds. Tune this — smaller = finer control, larger = faster sweep.
    private static final double MOVE_RATE = 0.01;

    // Dpad step sizes.
    private static final double BIG_STEP = 0.1;   // up / down
    private static final double SMALL_STEP = 0.01; // right / left

    // Previous-loop dpad states, used to detect a fresh press (rising edge)
    // so holding a dpad button doesn't repeat the nudge every tick.
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "kicker   ");
        servo.setPosition(position);

        telemetry.addLine("Initialized — press PLAY");
        telemetry.addLine("Left stick Y: sweep | Dpad U/D: ±0.1 | Dpad L/R: ±0.01");
        telemetry.addLine("A: 0.0 | B: 1.0 | X: 0.5");
        telemetry.update();
    }

    @Override
    public void loop() {

        double stick = -gamepad1.left_stick_y;   // Controller up = increase position
        if (Math.abs(stick) < STICK_DEADZONE) stick = 0;

        position += stick * MOVE_RATE;

        // Dpad — discrete nudges, one per press (rising edge only).
        if (gamepad1.dpad_up && !lastDpadUp) position += BIG_STEP;
        if (gamepad1.dpad_down && !lastDpadDown) position -= BIG_STEP;
        if (gamepad1.dpad_right && !lastDpadRight) position += SMALL_STEP;
        if (gamepad1.dpad_left && !lastDpadLeft) position -= SMALL_STEP;

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Snap-to-preset buttons override the stick/dpad for quick reference points.
        if (gamepad1.a) position = 0.0;
        if (gamepad1.b) position = 1.0;
        if (gamepad1.x) position = 0.5;

        // Clamp — servo positions outside 0.0–1.0 are invalid and some hubs
        // will throw or silently clip in unpredictable ways.
        position = Math.max(0.0, Math.min(1.0, position));

        servo.setPosition(position);

        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addData("Stick input", "%.2f", stick);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Leave the servo where it is — don't snap it back on stop, since the
        // whole point of this OpMode is finding and holding a position.
    }
}
