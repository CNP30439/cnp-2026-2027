package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// TeleMainTurret.java — TeleOp for the turret robot.
// Pedro manual drive + turret that keeps pointing at the goal (on/off switch)
// + shooter, intake, flower, blocker, kicker, prism.
//
// THIS REVISION:
//   * GAMEPAD 1 LEFT TRIGGER (L2) = TURRET KILL SWITCH. Press to cut ALL turret power (turret goes
//     limp), press again to bring it back. Backup for when the turret misbehaves.
//   * GAMEPAD 1 RIGHT TRIGGER (R2, hold) = AUTO-DRIVE TO THE SHOOTING ZONE, only while the turret is
//     "off" (L2 kill switch ON, or right bumper aim OFF). Logic copied from AutoShootDodgeRange:
//       - drives to the nearest legal point in the shooting zone (SHOOT_DISTANCE from the goal),
//         turning so the BACK of the robot (turret at zero) faces the goal
//       - routes around the two structures (KEEP_OUT_ZONES) with one smooth path
//       - then HOLDS the pose while R2 stays held, re-aims, and once the aim is locked it
//         AUTO-SHOOTS exactly like gamepad 2 R2 (shooter must already be on: gamepad 2 dpad up)
//       - the turret is LOCKED at zero (powered hold) the whole time R2 is held
//       - release R2 (or turn the turret back on) = driver control again; killed turret goes limp
//     NOT COPIED: the Limelight pose reset (this robot has no Limelight in its config).
//   * Anti-shake: kS ramps in smoothly (full at 1 deg), SPIN_FILTER 0.08.
//   * Side correction: right side AND left side within AIM_LEFT_DEADBAND_IN (25 in) use -4 deg;
//     left side beyond that uses -2 deg.
//   * SHOOT GATE: the kicker only fires once the turret is within SHOOT_READY_DEG of its target
//     (checked while tracking or auto-driving). Set SHOOT_READY_DEG to 180 to turn the gate off.
//   If it STILL shakes: lower kP (5.0 -> 3.5 -> 3.0), then lower kD (0.12 -> 0.08).
//   NOT FIXED (hardware choice): the 6 deg dead gap in the turret range (-87..267). When the goal
//   bearing is in the gap the turret parks at the nearest limit. Re-zero the start so the gap
//   points where you never shoot, or widen travel.
//
// HARDWARE (names must match the Robot Configuration)
//   fl, fr, bl, br (Pedro Constants) | ls shooter | li, ri intake (ri REVERSED)
//   linkageleft, linkageright (flower) | blocker | kicker | prism | turret (ELC encoder on its encoder port)
//
// INIT (gamepad 1): dpad up = RED, dpad down = BLUE. Turret must be at its zero (facing back) at INIT.
//
// GAMEPAD 1 (driver)
//   left stick = drive/strafe, right stick X = rotate
//   B (press)            = switch preset 0 <-> 1 (changes goal + reset pose)
//   X (press)            = reset pose to the preset's start pose (ignored while auto-driving)
//   right bumper (press) = TURRET AIM ON / OFF (OFF parks turret at zero)
//   L2 (press)           = TURRET KILL SWITCH (cuts all turret power, press again to restore)
//   R2 (hold)            = auto-drive to shooting zone + hold + auto-shoot (turret must be off)
//
// GAMEPAD 2 (operator)
//   dpad up      = shooter on / off
//   dpad left / right = shooter velocity -/+ 50 ticks/sec (starts at SHOOTER_START_VELOCITY)
//   A / B        = intake in / out (hold)
//   Y            = toggle flower up (0.7) / down (0.153)
//   R2 (hold)    = SHOOT: intake IN, blocker OPEN, kicker fires after KICK_DELAY_MS
//                  AND the turret is on target. Release: kicker back to BASE, blocker CLOSED.
//   NOTE: R2 does not spin the shooter up. Turn it on with dpad up first.
//
// GAME RAMP: from PLAY the commanded shooter velocity creeps up by GAME_RAMP_EXTRA over GAME_RAMP_SECS.
//
// SIDE AIM CORRECTION (turret tracking only, works for RED/BLUE and preset 0/1):
//   Red:  robot y < goal y = RIGHT side of goal, else LEFT.   Blue is mirrored.
//   Right side                          -> turret turns 4.0 deg right.
//   Left side, within 25 in (y) of goal -> turret turns 4.0 deg right (same as right side).
//   Left side, beyond 25 in             -> turret turns 2.0 deg right.
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@TeleOp(name = "TeleMain😎", group = "TeleOp")
public class TeleMainTurret extends OpMode {

    // ── Field data ───────────────────────────────────────────────────────────
    private enum Alliance { RED, BLUE }

    private static final PoseFactory p = PoseFactory.degrees();

    public static final double[][] RED_GOALS  = { {58, 52}, {58, 90} };
    public static final double[][] BLUE_GOALS = { {84, 51}, {84, 90} };

    public static final Pose[] RED_RESET_POSES = {
            p.of(7.25, 8, -90), p.of(7.25, 136, 90)
    };
    public static final Pose[] BLUE_RESET_POSES = {
            p.of(135.25, 8, -90), p.of(135.25, 136, 90)
    };

    // ── Drive ────────────────────────────────────────────────────────────────
    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Auto-drive to the shooting zone (gamepad 1 R2, turret off) ──────────
    private enum AlignState { MANUAL, ALIGNING, ARRIVED }

    // The robot may shoot from anywhere inside these bounds (R2 drives to the nearest point inside them).
    private static final double PRESET0_MAX_Y = 25;   // preset 0: y must be under this
    private static final double PRESET1_MIN_Y = 116;  // preset 1: y must be over this
    private static final double RED_X_MIN = 25, RED_X_MAX = 85;
    private static final double BLUE_X_MIN = 50, BLUE_X_MAX = 119;
    private static final double ZONE_MARGIN = 1.0;    // aim this far INSIDE the bounds
    private static final double WALL_MARGIN = 8.0;    // robot center stays this far from field edge

    // Distance from the goal the robot tries to stop at (the distance the shot is tuned for).
    // Set to 0 to go back to "nearest point in zone".
    private static final double SHOOT_DISTANCE = 41.0;
    // Gap from robot center to where balls leave (shooter is on the back). 0 = measure from the center.
    private static final double SHOOTER_OFFSET_IN = 0.0;

    // Turn-in-place arrival tolerances (when already inside the zone).
    private static final double HEADING_TOL_RAD = Math.toRadians(3);
    private static final double POSITION_TOL_IN = 1.5;

    // Aim check before auto-shoot (measured from where the robot IS, not where it was told to stop).
    private static final double AIM_OK_RAD = Math.toRadians(1.0);   // start shooting when aim error is under this
    private static final double AIM_LOST_RAD = Math.toRadians(1.5); // pause shooting if it gets worse than this
    private static final double AIM_LATE_RAD = Math.toRadians(1.5); // after AIM_LATE_MS, accept this instead
    private static final double AIM_LATE_MS = 1500;
    private static final double REAIM_RAD = Math.toRadians(0.3);    // re-aim the hold heading when it is off by this
    private static final double REAIM_EVERY_MS = 50;

    private AlignState alignState = AlignState.MANUAL;
    private boolean alignWasHeld = false;
    private boolean alignStarted = false; // follower became busy after scheduling (stops a false ARRIVED)
    private boolean turnOnly = false;
    private boolean aimLocked = false;
    private final ElapsedTime arrivedTimer = new ElapsedTime();
    private final ElapsedTime reaimTimer = new ElapsedTime();
    private Pose alignTarget = p.of(0, 0, 0);

    // ── Structure dodging ────────────────────────────────────────────────────
    // Two structures to avoid, each { minX, minY, maxX, maxY } in field inches.
    // A rectangle left as {0,0,0,0} is ignored.
    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91}, // STRUCTURE 1
            {91, 50, 96, 91}  // STRUCTURE 2
    };
    // The robot pose is its CENTER, so it must stay at least this far from a structure's edge.
    private static final double ROBOT_MARGIN = 7.0;
    // Blocked straight line -> one continuous rounded path (lines + curves) around the structures.
    private static final double CORNER_RADIUS = 10.0;
    private static final int CURVE_SAMPLES = 20; // per segment
    private static final double PAD_STEP = 1.5;
    private static final int PAD_TRIES = 10;
    private static final double FIELD_MIN = 0;
    private static final double FIELD_MAX = 144;
    private int routeCorners = 0; // detour guide points in the current route (0 = straight)

    // ── Turret tuning ────────────────────────────────────────────────────────
    private static final String TURRET_MOTOR = "turret";

    // ELC encoder: counts per ENCODER rev, and encoder revs per turret rev (25T encoder gear on 82T turret gear).
    private static final double TICKS_PER_MOTOR_REV      = 4000.0;
    private static final double MOTOR_REV_PER_TURRET_REV = 82.0 / 25.0;
    // +1 if turning the turret counter-clockwise (from above) makes raw ticks go UP, else -1.
    private static final double ENCODER_SIGN = 1.0;
    // +1 if positive motor power turns the turret toward + angle (CCW). Flip if it runs away when aim is ON.
    private static final double POWER_SIGN = 1.0;
    private static final double TICKS_PER_RAD =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV / (2 * Math.PI);

    private static final boolean TURRET_REVERSED = false;
    // Travel limits from the start (shooter facing back = 0). Total travel is MORE than 360 deg
    // (-95..275 = 370), so near the seam the goal can be reached two ways. The turret stays on the
    // way it is on and only swings back when it runs out of room, so it no longer flip-flops/unwinds
    // every time the goal bearing wobbles across the seam. Widen using the INIT "Lowest / highest seen".
    private static final double TURRET_MIN_RAD = Math.toRadians(-95);
    private static final double TURRET_MAX_RAD = Math.toRadians(275);
    // Turret angle at INIT in the turret's own frame (0 = it starts facing back).
    private static final double TURRET_START_RAD = Math.toRadians(0.0);
    // Direction the shooter points relative to robot forward when the turret is at zero. PI = backward.
    private static final double TURRET_ZERO_OFFSET_RAD = Math.PI;

    // Turret pivot relative to robot center, inches. x = forward, y = left.
    private static final double PIVOT_X_IN = 0.0;
    private static final double PIVOT_Y_IN = 0.0;

    private static final double kP = 5.0;    // power per rad of error
    private static final double kD = 0.12;   // power per rad/s of turret speed
    private static final double kV = 0.17;   // power per rad/s of wanted-angle change (feedforward)
    private static final double kS = 0.04;   // static friction power (ramps in, see KS_RAMP_RAD)
    // kS reaches full strength at this error and scales down linearly below it (no on/off chatter).
    private static final double KS_RAMP_RAD = Math.toRadians(1.0);
    private static final double MAX_POWER = 0.96;
    private static final double SWING_POWER = 0.96;  // cap while error > SWING_ERR_DEG
    private static final double SWING_ERR_DEG = 25;
    private static final double TOLERANCE_RAD = Math.toRadians(0.22); // only used for the ON TARGET label
    private static final double SPIN_FILTER = 0.08;  // 0..1, higher = less smoothing
    // Kicker only fires while the turret is this close to its target. 180 = gate off.
    private static final double SHOOT_READY_DEG = 2.0;
    private static final double SHOOT_READY_RAD = Math.toRadians(SHOOT_READY_DEG);

    // Aim correction by side of goal (negative = right, positive = left).
    private static final double AIM_RIGHT_SIDE_RAD = Math.toRadians(-4.0); // robot on right side was shooting far left
    private static final double AIM_LEFT_SIDE_RAD  = Math.toRadians(-2.0); // robot on left side was 1-2 deg left
    // On the left side, within this many inches (in y) of the goal center, use the RIGHT-side correction.
    private static final double AIM_LEFT_DEADBAND_IN  = 25.0;

    // ── Turret state ─────────────────────────────────────────────────────────
    private DcMotorEx turret;
    private boolean aimOn = true;
    private boolean turretKilled = false; // L2 kill switch: all turret power cut
    private final ElapsedTime dtTimer = new ElapsedTime();
    private double lastWanted = 0;
    private double lastTarget = 0;
    private double spinRate = 0;
    private double seenMinDeg = 0, seenMaxDeg = 0; // INIT limit finder

    // ── Alliance / preset ────────────────────────────────────────────────────
    private Alliance alliance = Alliance.RED;
    private int presetIndex = 0;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter; // single shooter motor "ls" (right shooter "rs" removed)
    private static final double SHOOTER_P = 71.5;
    private static final double SHOOTER_F_LEFT = 12.84;
    private static final double SHOOTER_START_VELOCITY = 200; // preset the shooter starts at
    private double shooterTarget = SHOOTER_START_VELOCITY;
    private static final double VELOCITY_STEP = 10;
    private static final double VELOCITY_MIN = 0;
    private boolean shooterOn = false;

    // Game-long ramp: 0 -> GAME_RAMP_EXTRA over GAME_RAMP_SECS from PLAY, then holds.
    private static final double GAME_RAMP_SECS = 120;
    private static final double GAME_RAMP_EXTRA = 25;
    private final ElapsedTime gameTimer = new ElapsedTime();

    // ── Intake ───────────────────────────────────────────────────────────────
    private DcMotorEx leftIntake, rightIntake;
    private static final double INTAKE_POWER = 1.0;

    // ── Flower (replaces the old linkage: same two servos, directions from FlowerServoTest) ──
    private Servo flowerA, flowerB;
    private static final String FLOWER_A_NAME = "linkageright"; // FORWARD
    private static final String FLOWER_B_NAME = "linkageleft";  // REVERSED
    private static final double FLOWER_DOWN = 0.153;
    private static final double FLOWER_UP = 0.7;
    private boolean flowerUp = false;

    // ── Blocker / kicker ─────────────────────────────────────────────────────
    private Servo blocker;
    private static final double BLOCKER_CLOSED = 1.0;
    private static final double BLOCKER_OPEN = 0.6;

    private Servo kicker;
    private static final double KICKER_BASE = 0.5; // flipped (was 0.8)
    private static final double KICKER_KICK = 0.8; // flipped (was 0.5)
    private static final long KICK_DELAY_MS = 500;
    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean lastShooting = false;

    // ── Prism ────────────────────────────────────────────────────────────────
    private GoBildaPrismDriver prism;

    // ── Shoot trigger ────────────────────────────────────────────────────────
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    private boolean lastB, lastX1, lastRB1, lastL2;
    private boolean lastDpadUp, lastDpadLeft, lastDpadRight;
    private boolean lastY2;

    // ── Helpers ──────────────────────────────────────────────────────────────
    private Pose resetPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    private double[] goal() {
        return (alliance == Alliance.RED ? RED_GOALS : BLUE_GOALS)[presetIndex];
    }

    /** Red: robot y < goal y = right side of goal. Blue is mirrored. Same rule for preset 0 and 1. */
    private boolean onRightSideOfGoal(Pose pose) {
        return (alliance == Alliance.RED) == (pose.y() < goal()[1]);
    }

    /** "Turret off" = kill switch on, or aim switched off. Only then may R2 auto-drive. */
    private boolean turretOff() {
        return turretKilled || !aimOn;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double angleDiff(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DRIVE_DEADZONE ? 0 : v;
    }

    private double turretAngleRad() {
        return TURRET_START_RAD + ENCODER_SIGN * turret.getCurrentPosition() / TICKS_PER_RAD;
    }

    private double turretSpeedRadPerSec() {
        return ENCODER_SIGN * turret.getVelocity() / TICKS_PER_RAD;
    }

    /** Turret angle (rad, turret frame) that points the shooter at the goal right now (trim not included). */
    private double wantedTurretAngle(Pose pose) {
        double h = pose.heading();
        double px = pose.x() + PIVOT_X_IN * Math.cos(h) - PIVOT_Y_IN * Math.sin(h);
        double py = pose.y() + PIVOT_X_IN * Math.sin(h) + PIVOT_Y_IN * Math.cos(h);
        double[] g = goal();
        double fieldAngle = Math.atan2(g[1] - py, g[0] - px);
        return wrap(fieldAngle - h - TURRET_ZERO_OFFSET_RAD);
    }

    /**
     * Picks which equivalent angle (+-360 deg) the turret goes to: stays continuous with last loop's
     * target, and only jumps (unwinds) when that would leave the turret's range.
     * If nothing fits (gap in the range), waits at the nearest limit.
     */
    private double fitToRange(double angle, double prevTarget) {
        double w = wrap(angle);
        double twoPi = 2 * Math.PI;
        double cont = w + Math.round((prevTarget - w) / twoPi) * twoPi;
        if (cont >= TURRET_MIN_RAD && cont <= TURRET_MAX_RAD) return cont;

        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double a = w + k * twoPi;
            if (a < TURRET_MIN_RAD || a > TURRET_MAX_RAD) continue;
            if (Double.isNaN(best) || Math.abs(a - prevTarget) < Math.abs(best - prevTarget)) best = a;
        }
        if (!Double.isNaN(best)) return best;
        return Math.max(TURRET_MIN_RAD, Math.min(TURRET_MAX_RAD, cont));
    }

    private void setFlowerPosition(double position) {
        flowerA.setPosition(position);
        flowerB.setPosition(position);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Shooting-zone helpers (from AutoShootDodgeRange)
    // ═════════════════════════════════════════════════════════════════════════

    /** Heading (radians) that points the back of the robot (turret at zero) at the goal from (x, y). */
    private double headingToGoal(double x, double y) {
        double[] g = goal();
        return Math.atan2(g[1] - y, g[0] - x) + TURRET_ZERO_OFFSET_RAD;
    }

    /**
     * Fastest legal shooting pose: the point inside the shooting zone nearest to the robot right now,
     * with the heading that lines the back of the robot up on the goal.
     */
    private Pose computeShootPose() {
        Pose cur = follower.pose();
        double xMin = alliance == Alliance.RED ? RED_X_MIN : BLUE_X_MIN;
        double xMax = alliance == Alliance.RED ? RED_X_MAX : BLUE_X_MAX;
        double x = clamp(cur.x(), xMin + ZONE_MARGIN, xMax - ZONE_MARGIN);
        double yLo = (presetIndex == 0) ? FIELD_MIN + WALL_MARGIN : PRESET1_MIN_Y + ZONE_MARGIN;
        double yHi = (presetIndex == 0) ? PRESET0_MAX_Y - ZONE_MARGIN : FIELD_MAX - WALL_MARGIN;
        double y = clamp(cur.y(), yLo, yHi); // fallback: nearest point in the zone

        // Back off to SHOOT_DISTANCE from the goal (keeping the same x) if that is possible.
        double[] g = goal();
        double dx = x - g[0];
        double centerDist = SHOOT_DISTANCE + SHOOTER_OFFSET_IN;
        if (centerDist > Math.abs(dx)) {
            double dy = Math.sqrt(centerDist * centerDist - dx * dx);
            double idealY = (presetIndex == 0) ? g[1] - dy : g[1] + dy;
            y = clamp(idealY, yLo, yHi);
        }
        return p.of(x, y, Math.toDegrees(headingToGoal(x, y)));
    }

    /** Aim error (radians) from the robot's CURRENT position: + = shot too far left. */
    private double aimErrorRad() {
        Pose now = follower.pose();
        return angleDiff(now.heading(), headingToGoal(now.x(), now.y()));
    }

    private void enterArrived() {
        alignState = AlignState.ARRIVED;
        aimLocked = false;
        arrivedTimer.reset();
        reaimTimer.reset();
    }

    /** True only in MANUAL. While ALIGNING the path runs; while ARRIVED Pedro holds the target. */
    private boolean driverHasControl() {
        return alignState == AlignState.MANUAL;
    }

    /**
     * Gamepad 1 R2 (hold, turret off) = auto-drive to the shooting zone.
     * MANUAL   — driver (plain stick drive)
     * ALIGNING — path is running
     * ARRIVED  — path done; Pedro HOLDS the target while R2 stays held, re-aims, then aim-lock gates shooting.
     * Release R2 (or turn the turret back on) to get driver control back.
     */
    private void updateAlign() {
        boolean alignHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD && turretOff();
        boolean rising = alignHeld && !alignWasHeld;
        alignWasHeld = alignHeld;

        switch (alignState) {

            case MANUAL:
                if (rising) {
                    alignTarget = computeShootPose();
                    Pose now = follower.pose();
                    turnOnly = Math.hypot(alignTarget.x() - now.x(), alignTarget.y() - now.y()) < 0.75;
                    if (turnOnly) {
                        schedule(hold(follower, alignTarget)); // already in the zone: hold and rotate onto the goal
                    } else {
                        scheduleRoute(alignTarget);
                    }
                    alignStarted = false;
                    alignState = AlignState.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!alignHeld) {
                    Scheduler.reset(); // clears ALL scheduled commands
                    alignState = AlignState.MANUAL;
                } else if (turnOnly) {
                    Pose now = follower.pose();
                    boolean headingOk = Math.abs(angleDiff(now.heading(), alignTarget.heading())) < HEADING_TOL_RAD;
                    boolean posOk = Math.hypot(alignTarget.x() - now.x(), alignTarget.y() - now.y()) < POSITION_TOL_IN;
                    if (headingOk && posOk) enterArrived(); // hold already scheduled
                } else if (follower.isBusy()) {
                    alignStarted = true; // path is genuinely running
                } else if (alignStarted) {
                    schedule(hold(follower, alignTarget)); // reached the target: now hold it
                    enterArrived();
                }
                // !started and !busy: command queued, not started yet. Wait.
                break;

            case ARRIVED:
                if (!alignHeld) {
                    Scheduler.reset(); // stops the hold
                    alignState = AlignState.MANUAL;
                    aimLocked = false;
                    break;
                }

                // Re-aim from where the robot actually is (it may have stopped a little off, or been pushed).
                Pose now = follower.pose();
                double want = headingToGoal(now.x(), now.y());
                if (Math.abs(angleDiff(want, alignTarget.heading())) > REAIM_RAD
                        && reaimTimer.milliseconds() > REAIM_EVERY_MS) {
                    alignTarget = p.of(alignTarget.x(), alignTarget.y(), Math.toDegrees(want));
                    Scheduler.reset();
                    schedule(hold(follower, alignTarget));
                    reaimTimer.reset();
                }
                // Only shoot while really aimed. Pauses (and the kick delay restarts) if bumped off aim.
                double aimErr = Math.abs(aimErrorRad());
                double okTol = arrivedTimer.milliseconds() > AIM_LATE_MS ? AIM_LATE_RAD : AIM_OK_RAD;
                if (!aimLocked && aimErr < okTol) aimLocked = true;
                else if (aimLocked && aimErr > Math.max(AIM_LOST_RAD, okTol)) aimLocked = false;
                break;
        }
    }

    /** Straight line from the robot's current pose to the given target (heading turns as it drives). */
    private Path pathTo(Pose target) {
        Pose currentPose = follower.pose();
        return line(currentPose, target).linear(currentPose, target);
    }

    /** Schedules the drive to the target: straight if clear, else one rounded path around the structures. */
    private void scheduleRoute(Pose target) {
        Pose start = follower.pose();
        routeCorners = 0;
        if (isBlocked(start.x(), start.y(), target.x(), target.y())) {
            Path route = buildCurvedPath(start, target);
            if (route != null) {
                schedule(follow(follower, route));
                return;
            }
        }
        schedule(follow(follower, pathTo(target))); // clear, or no safe route: straight
    }

    /**
     * Builds the dodge path: lines joined by quadratic-Bezier rounded corners, with the heading turning
     * steadily from the current heading to the target's. Returns null if no safe route exists.
     */
    private Path buildCurvedPath(Pose start, Pose target) {
        double pad = ROBOT_MARGIN + CORNER_RADIUS * 0.5; // rounding cuts in by at most ~R/2
        for (int attempt = 0; attempt < PAD_TRIES; attempt++, pad += PAD_STEP) {
            List<double[]> corners = findCorners(start, target, pad);
            if (corners == null || corners.isEmpty()) return null;

            List<double[]> pts = new ArrayList<>();
            pts.add(new double[]{start.x(), start.y()});
            pts.addAll(corners);
            pts.add(new double[]{target.x(), target.y()});

            List<double[][]> segs = roundedSegments(pts);
            if (!segmentsClear(segs, start, target)) continue; // cut too close: widen, retry

            // Heading: spread the turn along the path in proportion to distance.
            double h0 = start.heading();
            double delta = Math.atan2(Math.sin(target.heading() - h0), Math.cos(target.heading() - h0));
            double total = 0;
            for (double[][] sg : segs) total += controlLength(sg);

            List<Path> parts = new ArrayList<>();
            double run = 0;
            for (double[][] sg : segs) {
                double len = controlLength(sg);
                double hA = h0 + delta * (run / total);
                double hB = h0 + delta * ((run + len) / total);
                run += len;

                Pose a = p.of(sg[0][0], sg[0][1], Math.toDegrees(hA));
                Pose b = p.of(sg[sg.length - 1][0], sg[sg.length - 1][1], Math.toDegrees(hB));
                if (sg.length == 2) {
                    parts.add(line(a, b).linear(a, b));
                } else {
                    Pose m = p.of(sg[1][0], sg[1][1], Math.toDegrees(hA)); // corner = control point
                    parts.add(curve(a, m, b).linear(a, b));
                }
            }
            routeCorners = corners.size();
            return path(parts.toArray(new Path[0]));
        }
        return null;
    }

    /** Polyline start -> corners -> target as segments: straight runs (2 pts) and rounded corners (3 pts). */
    private List<double[][]> roundedSegments(List<double[]> pts) {
        List<double[][]> out = new ArrayList<>();
        double[] cur = pts.get(0);
        for (int k = 1; k < pts.size() - 1; k++) {
            double[] a = pts.get(k - 1), c = pts.get(k), b = pts.get(k + 1);
            double lenIn = Math.hypot(c[0] - a[0], c[1] - a[1]);
            double lenOut = Math.hypot(b[0] - c[0], b[1] - c[1]);
            double rIn = Math.min(CORNER_RADIUS, lenIn / 2);
            double rOut = Math.min(CORNER_RADIUS, lenOut / 2);
            double[] entry = {c[0] - (c[0] - a[0]) / lenIn * rIn, c[1] - (c[1] - a[1]) / lenIn * rIn};
            double[] exit = {c[0] + (b[0] - c[0]) / lenOut * rOut, c[1] + (b[1] - c[1]) / lenOut * rOut};
            if (Math.hypot(entry[0] - cur[0], entry[1] - cur[1]) > 0.01) {
                out.add(new double[][]{cur, entry});
            }
            out.add(new double[][]{entry, c, exit});
            cur = exit;
        }
        out.add(new double[][]{cur, pts.get(pts.size() - 1)});
        return out;
    }

    private static double controlLength(double[][] sg) {
        double len = 0;
        for (int i = 1; i < sg.length; i++) {
            len += Math.hypot(sg[i][0] - sg[i - 1][0], sg[i][1] - sg[i - 1][1]);
        }
        return len;
    }

    /**
     * Picks which padded zone corners the robot wraps around, in order. Visibility graph over start,
     * target and every usable corner, shortest path by Dijkstra. Returns null if no route exists.
     */
    private List<double[]> findCorners(Pose start, Pose target, double pad) {
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{start.x(), start.y()});
        nodes.add(new double[]{target.x(), target.y()});
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double[][] corners = {
                    {z[0] - pad, z[1] - pad},
                    {z[2] + pad, z[1] - pad},
                    {z[2] + pad, z[3] + pad},
                    {z[0] - pad, z[3] + pad}
            };
            for (double[] c : corners) {
                boolean onField = c[0] >= FIELD_MIN && c[0] <= FIELD_MAX
                        && c[1] >= FIELD_MIN && c[1] <= FIELD_MAX;
                if (onField && !insideAnyZone(c[0], c[1])) nodes.add(c);
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        java.util.Arrays.fill(dist, Double.MAX_VALUE);
        java.util.Arrays.fill(prev, -1);
        dist[0] = 0;
        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && dist[i] < Double.MAX_VALUE && (u == -1 || dist[i] < dist[u])) u = i;
            }
            if (u == -1) break;
            done[u] = true;
            for (int v = 0; v < n; v++) {
                if (done[v]) continue;
                double[] a = nodes.get(u), b = nodes.get(v);
                if (isBlocked(a[0], a[1], b[0], b[1])) continue;
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
            }
        }
        if (dist[1] == Double.MAX_VALUE) return null;

        List<Integer> chain = new ArrayList<>();
        for (int at = 1; at != -1; at = prev[at]) chain.add(at);
        Collections.reverse(chain); // 0 (start) ... 1 (target)
        List<double[]> out = new ArrayList<>();
        for (int i = 1; i < chain.size() - 1; i++) out.add(nodes.get(chain.get(i)));
        return out;
    }

    /** Samples every path segment and checks it never comes within ROBOT_MARGIN of a structure. */
    private boolean segmentsClear(List<double[][]> segs, Pose start, Pose target) {
        final double eps = 0.01;
        for (double[][] sg : segs) {
            for (int i = 0; i <= CURVE_SAMPLES; i++) {
                double[] pt = bezierPoint(sg, (double) i / CURVE_SAMPLES);
                for (double[] z : KEEP_OUT_ZONES) {
                    if (!active(z)) continue;
                    double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
                    double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
                    if (inside(start.x(), start.y(), x0, y0, x1, y1)
                            || inside(target.x(), target.y(), x0, y0, x1, y1)) continue;
                    if (inside(pt[0], pt[1], x0, y0, x1, y1)) return false;
                }
            }
        }
        return true;
    }

    /** Point on a Bezier segment (2 or 3 control points) at t in [0,1], by de Casteljau. */
    private static double[] bezierPoint(double[][] ctrl, double t) {
        int n = ctrl.length;
        double[] x = new double[n], y = new double[n];
        for (int i = 0; i < n; i++) { x[i] = ctrl[i][0]; y[i] = ctrl[i][1]; }
        for (int k = n - 1; k > 0; k--) {
            for (int i = 0; i < k; i++) {
                x[i] = (1 - t) * x[i] + t * x[i + 1];
                y[i] = (1 - t) * y[i] + t * y[i + 1];
            }
        }
        return new double[]{x[0], y[0]};
    }

    /** True if the segment a-b passes through any padded keep-out zone. */
    private boolean isBlocked(double ax, double ay, double bx, double by) {
        final double eps = 0.01; // so points exactly on a padded edge count as outside
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
            double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
            // If an end is already inside this zone, don't let that zone block the leg (or we could never leave).
            if (inside(ax, ay, x0, y0, x1, y1) || inside(bx, by, x0, y0, x1, y1)) continue;
            if (segmentHitsRect(ax, ay, bx, by, x0, y0, x1, y1)) return true;
        }
        return false;
    }

    private boolean insideAnyZone(double x, double y) {
        final double eps = 0.01;
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            if (inside(x, y, z[0] - ROBOT_MARGIN + eps, z[1] - ROBOT_MARGIN + eps,
                    z[2] + ROBOT_MARGIN - eps, z[3] + ROBOT_MARGIN - eps)) return true;
        }
        return false;
    }

    /** A zone left as {0,0,0,0} is a placeholder and is ignored. */
    private static boolean active(double[] z) {
        return !(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 0);
    }

    private static boolean inside(double x, double y, double x0, double y0, double x1, double y1) {
        return x > x0 && x < x1 && y > y0 && y < y1;
    }

    /** Liang–Barsky segment vs. axis-aligned rectangle test. */
    private static boolean segmentHitsRect(double ax, double ay, double bx, double by,
                                           double x0, double y0, double x1, double y1) {
        double t0 = 0, t1 = 1;
        double dx = bx - ax, dy = by - ay;
        double[] pp = {-dx, dx, -dy, dy};
        double[] qq = {ax - x0, x1 - ax, ay - y0, y1 - ay};
        for (int i = 0; i < 4; i++) {
            if (pp[i] == 0) {
                if (qq[i] < 0) return false;
            } else {
                double t = qq[i] / pp[i];
                if (pp[i] < 0) {
                    if (t > t1) return false;
                    if (t > t0) t0 = t;
                } else {
                    if (t < t0) return false;
                    if (t < t1) t1 = t;
                }
            }
        }
        return true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        alliance = Alliance.RED;
        presetIndex = 0;
        follower.setPose(resetPose());
        alignTarget = resetPose();
        alignState = AlignState.MANUAL;
        alignWasHeld = false;
        alignStarted = false;

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter = hardwareMap.get(DcMotorEx.class, "ls");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT));

        leftIntake = hardwareMap.get(DcMotorEx.class, "li");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flowerA = hardwareMap.get(Servo.class, FLOWER_A_NAME);
        flowerB = hardwareMap.get(Servo.class, FLOWER_B_NAME);
        flowerA.setDirection(Servo.Direction.FORWARD);
        flowerB.setDirection(Servo.Direction.REVERSE);
        flowerUp = true; // flower starts UP at INIT
        setFlowerPosition(FLOWER_UP);

        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setPosition(BLOCKER_CLOSED);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_BASE);

        telemetry.addLine("Gamepad 1: dpad UP = RED, dpad DOWN = BLUE");
        telemetry.addLine("Turret must be at its zero position NOW (encoder is zeroed in INIT)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;
        follower.setPose(resetPose());

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose", "x=%.1f y=%.1f h=%.0f deg",
                resetPose().x(), resetPose().y(), Math.toDegrees(resetPose().heading()));
        // LIMIT FINDER: turn the turret by hand to each hard stop and read the angle.
        double angle = Math.toDegrees(turretAngleRad());
        seenMinDeg = Math.min(seenMinDeg, angle);
        seenMaxDeg = Math.max(seenMaxDeg, angle);
        telemetry.addData("Turret angle", "%.1f deg   raw ticks %d", angle, turret.getCurrentPosition());
        telemetry.addData("Lowest / highest seen", "%.1f / %.1f deg", seenMinDeg, seenMaxDeg);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(resetPose());
        alignTarget = resetPose();
        lastWanted = wantedTurretAngle(follower.pose());
        lastTarget = turretAngleRad();
        gameTimer.reset();
        dtTimer.reset();
        loopTimer.reset();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // ── Gamepad 1 buttons (rising edges) ─────────────────────────────
        boolean b = gamepad1.b, x1 = gamepad1.x, rb1 = gamepad1.right_bumper;
        boolean l2 = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        if (b && !lastB) presetIndex = 1 - presetIndex;
        if (x1 && !lastX1 && driverHasControl()) follower.setPose(resetPose());
        if (rb1 && !lastRB1) aimOn = !aimOn;
        if (l2 && !lastL2) turretKilled = !turretKilled; // L2 = turret kill switch
        lastB = b; lastX1 = x1; lastRB1 = rb1; lastL2 = l2;

        // ── Auto-drive to shooting zone (gamepad 1 R2, turret off) ────────
        updateAlign();

        // ── Drive ────────────────────────────────────────────────────────
        // Pedro: +lateral = left, +turn = CCW. Sticks are + to the right, so negate.
        if (driverHasControl()) {
            follower.manual(deadzone(-gamepad1.left_stick_y),
                    -deadzone(gamepad1.left_stick_x),
                    -deadzone(gamepad1.right_stick_x));
        }
        follower.update();
        Scheduler.execute();

        // ── Turret ───────────────────────────────────────────────────────
        Pose pose = follower.pose();
        double dt = Math.max(dtTimer.seconds(), 1e-3);
        dtTimer.reset();
        // Rate the wanted angle changes (robot spin + translation), filtered.
        double wanted = wantedTurretAngle(pose);
        double rawRate = wrap(wanted - lastWanted) / dt;
        lastWanted = wanted;
        spinRate += SPIN_FILTER * (rawRate - spinRate);

        // Right side: always -4 deg. Left side: -4 deg within 25 in of goal y, else -2 deg.
        boolean rightSide = onRightSideOfGoal(pose);
        boolean nearLeft = !rightSide && Math.abs(pose.y() - goal()[1]) <= AIM_LEFT_DEADBAND_IN;
        double sideOffset = (rightSide || nearLeft) ? AIM_RIGHT_SIDE_RAD : AIM_LEFT_SIDE_RAD;

        // tracking = turret follows the goal. Otherwise it is held at zero (parked / locked for auto-drive),
        // or, if killed and not auto-driving, left with no power at all.
        boolean tracking = aimOn && !turretKilled;
        boolean autoDriving = alignState != AlignState.MANUAL;
        boolean powerCut = turretKilled && !autoDriving;

        double current = turretAngleRad();
        double target = tracking ? fitToRange(wanted + sideOffset, lastTarget) : 0;
        lastTarget = tracking ? target : current; // tracking turned back ON starts from where the turret really is
        double err = target - current;

        double power = 0;
        if (!powerCut) {
            // kS ramps in smoothly (full at KS_RAMP_RAD) so there is no on/off step at the target.
            double ksRamp = Math.max(-1.0, Math.min(1.0, err / KS_RAMP_RAD));
            power = kP * err - kD * turretSpeedRadPerSec() + ksRamp * kS;
            if (tracking) power += kV * spinRate; // follows how fast the goal bearing changes (robot turning / driving)
            double cap = Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? SWING_POWER : MAX_POWER;
            power = Math.max(-cap, Math.min(cap, power));
        }
        turret.setPower(POWER_SIGN * power);

        // Ready to shoot: manual with turret off = always; tracking or auto-driving = within SHOOT_READY_RAD.
        boolean turretReady = (tracking || autoDriving) ? Math.abs(err) <= SHOOT_READY_RAD : true;

        // ── Shooter mode (gamepad 2 dpad) ────────────────────────────────
        boolean dUp = gamepad2.dpad_up, dLeft = gamepad2.dpad_left, dRight = gamepad2.dpad_right;
        if (dUp && !lastDpadUp) shooterOn = !shooterOn;
        if (dRight && !lastDpadRight) shooterTarget += VELOCITY_STEP;
        if (dLeft && !lastDpadLeft) shooterTarget = Math.max(VELOCITY_MIN, shooterTarget - VELOCITY_STEP);
        lastDpadUp = dUp; lastDpadRight = dRight; lastDpadLeft = dLeft;

        // Shoot when gamepad 2 R2 is held, OR automatically once the robot has arrived at the shooting
        // zone with gamepad 1 R2 still held and the aim is locked.
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD
                || (alignState == AlignState.ARRIVED && aimLocked);

        double shooterVelocity = shooterOn ? shooterTarget : 0;
        String shooterMode = shooterOn ? "ON" : "OFF";

        double gameRamp = GAME_RAMP_EXTRA * Math.min(1.0, gameTimer.seconds() / GAME_RAMP_SECS);
        double commandedVelocity = shooterOn ? (shooterVelocity + gameRamp) : 0;
        leftShooter.setVelocity(commandedVelocity);

        // ── SHOOT (held) ─────────────────────────────────────────────────
        if (shooting && !lastShooting) kickTimer.reset();
        lastShooting = shooting;
        boolean kickReady = shooting && turretReady && kickTimer.milliseconds() >= KICK_DELAY_MS;

        double intakePower;
        String intakeMode;
        if (shooting) {
            intakePower = INTAKE_POWER;
            intakeMode = "SHOOT-FEED";
            blocker.setPosition(BLOCKER_OPEN);
            kicker.setPosition(kickReady ? KICKER_KICK : KICKER_BASE);
        } else {
            if (gamepad2.a) {
                intakePower = INTAKE_POWER;
                intakeMode = "IN";
            } else if (gamepad2.b) {
                intakePower = -INTAKE_POWER;
                intakeMode = "OUT";
            } else {
                intakePower = 0;
                intakeMode = "STOPPED";
            }
            blocker.setPosition(BLOCKER_CLOSED);
            kicker.setPosition(KICKER_BASE);
        }
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // ── Flower (gamepad 2 Y toggle) ──────────────────────────────────
        boolean y2 = gamepad2.y;
        if (y2 && !lastY2) {
            flowerUp = !flowerUp;
            setFlowerPosition(flowerUp ? FLOWER_UP : FLOWER_DOWN);
        }
        lastY2 = y2;

        // ── Telemetry ────────────────────────────────────────────────────
        double[] g = goal();
        telemetry.addData("Loop time", "%.1f ms", loopMs);
        telemetry.addData("Shooter", "%s target=%.0f commanded=%.0f actual=%.0f",
                shooterMode, shooterVelocity, commandedVelocity, leftShooter.getVelocity());
        telemetry.addData("Shooter set velocity", "%.0f", shooterTarget);
        telemetry.addData("Game ramp", "+%.1f ticks/sec (%.0f s elapsed)", gameRamp, gameTimer.seconds());
        telemetry.addData("Shooting", shooting ? (gamepad2.right_trigger > TRIGGER_THRESHOLD ? "YES (G2 R2)" : "YES (auto)") : "no");
        if (shooting) {
            telemetry.addData("Kick delay", "%.0f / %d ms %s",
                    Math.min(kickTimer.milliseconds(), KICK_DELAY_MS), KICK_DELAY_MS,
                    kickReady ? "(FIRING)" : (turretReady ? "(waiting)" : "(WAITING FOR TURRET)"));
        }
        telemetry.addData("Intake", intakeMode);
        telemetry.addData("Blocker", shooting ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker", kickReady ? "KICK" : "BASE");
        telemetry.addData("Flower", flowerUp ? "UP" : "DOWN");
        telemetry.addLine("---");
        telemetry.addData("Alliance / preset", "%s #%d  goal=(%.0f, %.0f)", alliance, presetIndex, g[0], g[1]);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to goal", "%.1f in", Math.hypot(g[0] - pose.x(), g[1] - pose.y()));
        telemetry.addData("Goal side", "%s  offset=%.1f deg",
                rightSide ? "RIGHT" : (nearLeft ? "LEFT (near)" : "LEFT"), Math.toDegrees(sideOffset));
        telemetry.addData("TURRET", turretKilled ? (autoDriving ? "KILLED (locked for auto-drive)" : "KILLED (no power)")
                : (aimOn ? "AIM ON" : "AIM OFF (parked)"));
        telemetry.addData("Auto-drive (G1 R2)", "%s busy=%b route=%s", alignState, follower.isBusy(),
                alignState == AlignState.MANUAL ? "-" : (routeCorners > 0 ? "curving (" + routeCorners + " pts)" : "straight"));
        if (autoDriving) {
            telemetry.addData("Shoot target", "x=%.1f y=%.1f h=%.0f deg",
                    alignTarget.x(), alignTarget.y(), Math.toDegrees(alignTarget.heading()));
            telemetry.addData("Aim error", "%.2f deg %s", Math.toDegrees(aimErrorRad()),
                    alignState == AlignState.ARRIVED ? (aimLocked ? "(LOCKED - shooting)" : "(waiting to line up)") : "");
        }
        telemetry.addData("Turret", "now=%.1f target=%.1f err=%.2f deg %s",
                Math.toDegrees(current), Math.toDegrees(target), Math.toDegrees(err),
                powerCut ? "(NO POWER)" : (!tracking ? "(parked)" : Math.abs(err) <= TOLERANCE_RAD ? "(ON TARGET)"
                        : (Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? "(far off)" : "")));
        double goalBearing = Math.atan2(g[1] - pose.y(), g[0] - pose.x());
        telemetry.addData("Turret heading", "%.1f deg (field)  goal bearing %.1f deg",
                Math.toDegrees(wrap(pose.heading() + current + TURRET_ZERO_OFFSET_RAD)), Math.toDegrees(goalBearing));
        telemetry.addData("Turret raw ticks", "%d", turret.getCurrentPosition());
        telemetry.addData("Goal rate / power", "%.0f deg/s  %.2f", Math.toDegrees(spinRate), power);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (turret != null) turret.setPower(0);
        leftShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        setFlowerPosition(FLOWER_DOWN);
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);
    }
}
