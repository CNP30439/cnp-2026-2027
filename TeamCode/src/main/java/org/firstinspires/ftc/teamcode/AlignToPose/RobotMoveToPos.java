package org.firstinspires.ftc.teamcode.AlignToPose;

// ═══════════════════════════════════════════════════════════════════════════════
//  RobotMoveToPos.java — one-file TeleOp: drive normally, hold R2 to auto-drive
//  to a fixed pose, press X to reset the pose to (0, 0, 0°).
//
//  FILE NAME MUST BE:  RobotMoveToPos.java
//  FOLDER MUST BE:     .../teamcode/AlignToPose/   (matches the package line)
//
//  CONTROLS (gamepad1)
//    Sticks — drive. Moving a stick during an alignment cancels it.
//    R2     — hold to auto-drive to IDEAL_POSE. Release to cancel.
//    X      — reset pose to ORIGIN. Ignored while aligning.
//
//  COORDINATE FRAME
//    Local frame, not the standard Pedro field map. Robot starts at ORIGIN
//    (0, 0, 0°); IDEAL_POSE is measured from that same spot. Press X with the
//    robot back on the reset spot, facing the same way, to fix drift.
//    To set IDEAL_POSE: drive the robot by hand to the target, read X / Y /
//    Heading from telemetry, and copy them into IDEAL_POSE below.
//
//  STATES
//    MANUAL   — driver owns the drivetrain.
//    ALIGNING — Ivy owns it, driving the path to IDEAL_POSE.
//    ARRIVED  — path done. Held until R2 is released so one press = one run.
//
//  REQUIRES IVY (https://pedropathing.com/docs/ivy).
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@TeleOp(name = "Align to pose")
public class RobotMoveToPos extends OpMode {

    public enum State { MANUAL, ALIGNING, ARRIVED }

    private static final PoseFactory p = PoseFactory.degrees();

    // Reset / starting spot.
    public static final Pose ORIGIN = p.of(8, 8, 90);

    // Target, measured from ORIGIN. PLACEHOLDER — replace with numbers read
    // from telemetry after driving the robot to the real spot by hand.
    public static final Pose IDEAL_POSE = p.of(56, 14, -90);

    // Stick movement past this cancels an in-progress alignment.
    private static final double STICK_ABORT_THRESHOLD = 0.15;

    // R2 is an analog trigger (0 to 1). Count it as "held" past this value.
    private static final double TRIGGER_THRESHOLD = 0.5;

    private Follower follower;

    private State   state        = State.MANUAL;
    private boolean wasHeld      = false;   // R2 last tick
    private boolean wasResetHeld = false;   // X last tick

    // True once the follower has actually become busy after scheduling.
    // Without this, the first tick after R2 could see isBusy()==false (the
    // command is queued but not started) and wrongly jump to ARRIVED.
    private boolean started = false;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(ORIGIN);
        state = State.MANUAL;
        wasHeld = false;
        wasResetHeld = false;
        started = false;
    }

    @Override
    public void loop() {
        handleControls();

        follower.update();
        Scheduler.execute();

        Pose pose = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Follower mode", follower.mode());
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to target", distanceToTarget());
        telemetry.update();
    }

    /** Reads X, R2 and the sticks, steps the state machine, drives manually when allowed. */
    private void handleControls() {
        // X: reset pose once per press, only while the driver has control.
        boolean xHeld = gamepad1.x;
        if (xHeld && !wasResetHeld && driverHasControl()) {
            follower.setPose(ORIGIN);
        }
        wasResetHeld = xHeld;

        double stickMag = Math.max(
                Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y)),
                Math.abs(gamepad1.right_stick_x));

        boolean r2Held = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean rising = r2Held && !wasHeld;
        wasHeld = r2Held;

        switch (state) {

            case MANUAL:
                // Only a fresh press starts an alignment.
                if (rising) {
                    schedule(follow(follower, pathToIdeal()));
                    started = false;
                    state = State.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!r2Held || stickMag > STICK_ABORT_THRESHOLD) {
                    // Driver let go or grabbed the sticks.
                    Scheduler.reset();   // clears ALL scheduled Ivy commands
                    state = State.MANUAL;
                } else if (follower.isBusy()) {
                    started = true;      // path is genuinely running
                } else if (started) {
                    state = State.ARRIVED;   // was running, now isn't → done
                }
                // !started and !isBusy: command queued, not started yet. Wait.
                break;

            case ARRIVED:
                // Driver already has the sticks. Wait for R2 release so a held
                // trigger doesn't re-fire.
                if (!r2Held) {
                    state = State.MANUAL;
                }
                break;
        }

        if (driverHasControl()) {
            // Pedro: positive lateral = left, positive turn = counterclockwise.
            // Sticks are positive to the right, so both are negated.
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }

    /** True when sticks should drive. False only while Ivy owns the drivetrain. */
    private boolean driverHasControl() {
        return state != State.ALIGNING;
    }

    /**
     * Straight line from the robot's current pose to IDEAL_POSE, with linear
     * heading interpolation.
     *
     * TO ROUTE AROUND A FIXED OBSTACLE: swap line() for curve() and add a
     * control pose, e.g.
     *     Pose control = p.of(60, 40, 90);
     *     return curve(currentPose, control, IDEAL_POSE).linear(currentPose, IDEAL_POSE);
     * (add `curve` to the static imports).
     */
    private Path pathToIdeal() {
        Pose currentPose = follower.pose();
        return line(currentPose, IDEAL_POSE).linear(currentPose, IDEAL_POSE);
    }

    /** Distance in inches from the robot to the target, for telemetry. */
    private double distanceToTarget() {
        return Math.hypot(IDEAL_POSE.x() - follower.pose().x(),
                IDEAL_POSE.y() - follower.pose().y());
    }
}