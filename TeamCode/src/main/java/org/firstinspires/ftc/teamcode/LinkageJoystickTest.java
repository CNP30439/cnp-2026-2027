package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  LinkageJoystickTest.java  —  Bench-test OpMode: drive the TWO mirrored
//                                linkage servos with a joystick, always in
//                                sync. Useful for finding the linkage's real
//                                UP/DOWN positions before hardcoding them.
//
//  HARDWARE
//    2x servos — "linkageLeft", "linkageRight"   (mirrored, same position)
//
//  CONTROLS
//    Gamepad 1
//      left stick Y   — moves BOTH linkage servos' position up/down together
//                        (rate-based — hold to sweep, ease off to land exactly)
//      A              — snap linkage to position 0.0  (DOWN)
//      B              — snap linkage to position 1.0  (UP)
//
//  WHY ONE STICK DRIVES TWO SERVOS
//    Same idea as the linkage helper in TeleOpBasic: these two servos are
//    mirrored on either side of the mechanism and must always match, so
//    there's a single `position` value and a single setLinkagePosition()
//    helper that writes it to both — never set them separately or they'll
//    drift out of sync.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LinkageJoystickTest", group = "Test")
public class LinkageJoystickTest extends OpMode {

    private Servo linkageLeft, linkageRight;

    // Current commanded position, tracked in software since Servo has no
    // reliable getPosition() readback on most hubs.
    private double position = 0.0;

    // Stick values below this are treated as zero, so a resting stick doesn't
    // slowly drift the linkage from residual noise.
    private static final double STICK_DEADZONE = 0.05;

    // How far the position moves per loop tick at full stick deflection.
    // Loop runs roughly every 20ms, so 0.01 here is a full sweep (0 to 1) in
    // about 2 seconds. Tune this — smaller = finer control, larger = faster sweep.
    private static final double MOVE_RATE = 0.01;

    @Override
    public void init() {
        linkageLeft  = hardwareMap.get(Servo.class, "linkageLeft");
        linkageRight = hardwareMap.get(Servo.class, "linkageRight");

        // If the two servos are mounted mirrored (facing opposite ways), one
        // of them needs REVERSE here so a single position value still drives
        // both to the matching physical spot. Uncomment if yours are mirrored:
        // linkageRight.setDirection(Servo.Direction.REVERSE);

        setLinkagePosition(position);   // Park at 0.0 (DOWN) on init

        telemetry.addLine("Initialized — press PLAY");
        telemetry.addLine("Left stick Y: sweep | A: 0.0 (DOWN) | B: 1.0 (UP)");
        telemetry.update();
    }

    @Override
    public void loop() {

        double stick = -gamepad1.left_stick_y;   // Controller up = increase position
        if (Math.abs(stick) < STICK_DEADZONE) stick = 0;

        position += stick * MOVE_RATE;

        // Snap-to-preset buttons override the stick for quick reference points.
        if (gamepad1.a) position = 0.0;
        if (gamepad1.b) position = 1.0;

        // Clamp — servo positions outside 0.0–1.0 are invalid and some hubs
        // will throw or silently clip in unpredictable ways.
        position = Math.max(0.0, Math.min(1.0, position));

        setLinkagePosition(position);

        telemetry.addData("Linkage position", "%.3f", position);
        telemetry.addData("Stick input", "%.2f", stick);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Leave the servos where they are — don't snap back on stop, since the
        // whole point of this OpMode is finding and holding a position.
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Helper — drives BOTH linkage servos to the same position, always.
    //  Never set linkageLeft/linkageRight individually outside this method.
    // ═════════════════════════════════════════════════════════════════════════
    private void setLinkagePosition(double pos) {
        linkageLeft.setPosition(pos);
        linkageRight.setPosition(pos);
    }
}