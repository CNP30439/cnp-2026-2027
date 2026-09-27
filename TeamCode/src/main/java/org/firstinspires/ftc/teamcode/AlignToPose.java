package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  AlignToPose.java  —  Pedro 3 "drive to a fixed pose" helper, with explicit
//                       ownership of the drivetrain via a small state machine.
//
//  Docs this is built from:
//    https://pedropathing.com/docs/pathing/reference/posefactory
//    https://pedropathing.com/docs/pathing/reference/api
//    https://pedropathing.com/docs/pathing/reference/interpolation
//    https://pedropathing.com/docs/pathing/guide/path-following
//    https://pedropathing.com/docs/pathing/guide/follow-state
//
//  WHY A STATE MACHINE
//  ──────────────────────────────────────────────────────────────────────────────
//  The previous version scheduled a path on the R1 rising edge and never
//  cancelled it. That left three holes:
//    • Release R1 mid-drive → Ivy kept driving while the OpMode also started
//      calling follower.manual(), so two things fought for the drivetrain.
//    • Press R1 again mid-drive → a second follow command got scheduled.
//    • Arrive at the target → nothing handed control back, so the sticks
//      stayed dead.
//  Now there are exactly three states and only one owner of the drivetrain at
//  any moment. The OpMode asks driverHasControl() and only calls manual() when
//  that returns true.
//
//  STATES
//    MANUAL   — driver owns the drivetrain. Default, and where we return to.
//    ALIGNING — Ivy owns it, driving the scheduled path to IDEAL_POSE.
//    ARRIVED  — path finished. Held until R1 is released, so one press does
//               one alignment and doesn't immediately re-fire.
//
//  ABORTS (any of these drop straight back to MANUAL):
//    • R1 released
//    • Driver touches a stick past STICK_ABORT_THRESHOLD — the practical
//      answer to "another robot is in the way"
//    • Path finishes → ARRIVED
//
//  REQUIRES IVY (https://pedropathing.com/docs/ivy).
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public class AlignToPose {

    public enum State { MANUAL, ALIGNING, ARRIVED }

    // PoseFactory in DEGREES, so headings are plain degrees (90) rather than
    // Math.toRadians(90). Also gives you mirrorX(70.75) / mirrorY(70.75) later
    // if you want the alliance-mirrored version of this target for free.
    private static final PoseFactory p = PoseFactory.degrees();

    // Ideal position (100, 20) facing 90°.
    public static final Pose IDEAL_POSE = p.of(100, 20, 90);

    // Stick movement past this cancels an in-progress alignment and returns
    // control to the driver immediately.
    private static final double STICK_ABORT_THRESHOLD = 0.15;

    private State   state   = State.MANUAL;
    private boolean wasHeld = false;

    /**
     * Call once per loop tick, before you decide whether to drive manually.
     *
     * @param follower the Pedro follower
     * @param r1Held   gamepad1.right_bumper
     * @param stickMag largest absolute stick value this tick — used to let the
     *                 driver abort an alignment by grabbing the sticks
     */
    public void update(Follower follower, boolean r1Held, double stickMag) {
        boolean rising = r1Held && !wasHeld;
        wasHeld = r1Held;

        switch (state) {

            case MANUAL:
                // Only a fresh press starts an alignment. Holding R1 down from
                // a previous alignment won't re-trigger — you must release
                // and press again.
                if (rising) {
                    schedule(follow(follower, pathToIdeal(follower)));
                    state = State.ALIGNING;
                }
                break;

            case ALIGNING:
                // Driver bailed, or grabbed the sticks to dodge something.
                if (!r1Held || stickMag > STICK_ABORT_THRESHOLD) {
                    abort();
                    state = State.MANUAL;
                } else if (!follower.isBusy()) {
                    // Path complete and settled at the target.
                    state = State.ARRIVED;
                }
                break;

            case ARRIVED:
                // Sit here until R1 is released, so a held button doesn't
                // immediately schedule another run at the same target.
                if (!r1Held) {
                    state = State.MANUAL;
                }
                break;
        }
    }

    /**
     * True when the OpMode should be calling follower.manual(). False while
     * Ivy owns the drivetrain — calling manual() then would fight the follower.
     *
     * Note ARRIVED returns true: the path is done, so the driver gets the
     * sticks back straight away even if they're still holding R1.
     */
    public boolean driverHasControl() {
        return state != State.ALIGNING;
    }

    /**
     * Cancel any in-progress alignment.
     *
     * NOTE: I could not confirm Ivy's per-command cancel API from the docs —
     * they cover schedule(), execute() and reset() but not cancellation of a
     * single command. Scheduler.reset() clears ALL scheduled commands, which
     * is correct here only because this OpMode schedules nothing else. If you
     * later add other Ivy commands, this will wipe them too — swap it for
     * Ivy's real cancel call once you find it in the Ivy source.
     */
    private void abort() {
        Scheduler.reset();
    }

    /**
     * Straight line from the robot's current pose to IDEAL_POSE, with linear
     * heading interpolation so it rotates smoothly to 90° across the drive.
     *
     * TO ROUTE AROUND A FIXED FIELD OBSTACLE: swap line() for curve() and add
     * a control pose that bows the path around it, e.g.
     *     Pose control = p.of(120, 60, 90);
     *     return curve(currentPose, control, IDEAL_POSE).linear(currentPose, IDEAL_POSE);
     * (add `curve` to the static imports). Control points pull the curve
     * toward them without the robot passing exactly through them.
     */
    private Path pathToIdeal(Follower follower) {
        Pose currentPose = follower.pose();
        return line(currentPose, IDEAL_POSE).linear(currentPose, IDEAL_POSE);
    }

    public State  state()                    { return state; }
    public double dx(Follower f)             { return IDEAL_POSE.x() - f.pose().x(); }
    public double dy(Follower f)             { return IDEAL_POSE.y() - f.pose().y(); }

    /** Distance in inches from the robot to the target, for telemetry. */
    public double distanceToTarget(Follower f) {
        return Math.hypot(dx(f), dy(f));
    }
}