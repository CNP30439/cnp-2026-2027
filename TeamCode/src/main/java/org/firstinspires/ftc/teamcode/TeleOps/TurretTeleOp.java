package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// TurretTeleOp.java — drive (Pedro, manual) + turret that keeps pointing at the goal.
//
// HOW THE AIM WORKS (every loop)
//   1. Pose comes from Pedro (x, y, heading).
//   2. Turret pivot position on the field = robot pose + pivot offset rotated by heading.
//   3. fieldAngle = atan2(goal - pivot).  Turret angle wanted = fieldAngle - robotHeading - TURRET_ZERO_OFFSET_RAD.
//   4. That angle is fitted into the turret's allowed range (cable limit): it picks the equivalent
//      angle (+-360 deg) that is in range and closest to where the turret is now; else clamps.
//   5. Control = P on angle error  - D on turret speed  + feedforward on robot spin  + small static boost.
//      The spin feedforward is the big one: when the driver turns the robot, the turret is told to
//      counter-rotate at the same rate, so it does not lag behind and aim stays on the goal.
//
// CONTROLS (gamepad 1)
//   left stick = drive/strafe, right stick X = rotate
//   B (press)  = switch preset 0 <-> 1 (changes goal)
//   X          = reset pose to the preset's start pose
//   right bumper (press) = turret aim ON / OFF (OFF parks turret at zero)
//   dpad left / right    = trim aim by -/+ 1 deg (for fixing a shot that is a little left/right)
// INIT: gamepad 1 dpad up = RED, dpad down = BLUE.
//
// HARDWARE: motor "turret" with encoder. Turret must be at its zero (pointing the way
// TURRET_ZERO_OFFSET_RAD says) when INIT is pressed: encoder is zeroed there.
//
// TUNE FIRST (all in the TUNING block): TICKS_PER_MOTOR_REV, MOTOR_REV_PER_TURRET_REV,
// TURRET_MIN/MAX_DEG, TURRET_ZERO_OFFSET_RAD, pivot offset, then kP, kD, kV, kS.
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TurretTeleOp", group = "TeleOp")
public class TurretTeleOp extends OpMode {

    // ── Field data (same numbers as the main TeleOp) ─────────────────────────
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

    // ── TUNING ───────────────────────────────────────────────────────────────
    private static final String TURRET_MOTOR = "turret";

    // The turret angle now comes from the ELC encoder (plugged into the turret motor's encoder port),
    // NOT the motor's own encoder. Its 25-tooth gear meshes the 22-tooth motor gear, which drives the 82-tooth turret gear.
    private static final double TICKS_PER_MOTOR_REV    = 4000.0; // ELC encoder quadrature, counts per ENCODER rev (name kept)
    private static final double MOTOR_REV_PER_TURRET_REV = 82.0 / 25.0; // ENCODER revs per turret rev = 3.28 (name kept)
    // +1 if turning the turret counter-clockwise (seen from above) makes the INIT "raw ticks" go UP, else -1.
    // Find it with the INIT test. A wrong sign makes the turret run away, so test with aim OFF first.
    private static final double ENCODER_SIGN = 1.0;
    // +1 if positive motor power turns the turret toward + angle (counter-clockwise). If the turret runs away
    // the moment aim is ON (it pushes away from the target), flip this. Keep a hand on STOP for this test.
    private static final double POWER_SIGN = 1.0;
    private static final double TICKS_PER_RAD =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV / (2 * Math.PI);

    // Turret angle 0 = encoder zero. + = counter-clockwise seen from above (same as robot heading).
    // If the motor makes + ticks turn the turret clockwise, flip TURRET_REVERSED.
    private static final boolean TURRET_REVERSED = false;
    // Range measured from the start (shooter facing the back, angle 0): 200 deg clockwise (-) and 85 deg
    // counter-clockwise (+). The turret MUST start facing exactly back.
    private static final double TURRET_MIN_RAD = Math.toRadians(-87);  // hard stop measured at -90, 3 deg margin
    private static final double TURRET_MAX_RAD = Math.toRadians(267);  // hard stop measured at +270, 3 deg margin

    // Turret angle at INIT, in the turret's own frame where 0 = the middle of its range (limits below are around 0).
    // 0   = you start it in the MIDDLE of its 180 deg range.
    // -90 = you start it pushed against the clockwise (right) hard stop; +90 = against the counter-clockwise (left) stop.
    // Starting against a hard stop every time is the only way the +-85 limits match the real stops.
    private static final double TURRET_START_RAD = Math.toRadians(0.0);

    // Direction the shooter points (relative to robot forward) when the turret is at its zero.
    // 0 = forward. Math.PI = backward (last year's robot had the shooter on the back).
    private static final double TURRET_ZERO_OFFSET_RAD = Math.PI;

    // Turret pivot position relative to robot center, in inches. x = forward, y = left.
    private static final double PIVOT_X_IN = 0.0;
    private static final double PIVOT_Y_IN = 0.0;

    private static final double kP = 3.0;     // power per rad of error
    private static final double kD = 0.08;    // power per rad/s of turret speed (damps overshoot)
    private static final double kV = 0.12;     // power per rad/s of wanted-angle change (feedforward, the lag killer).
    // Should be positive. If the turret runs AHEAD of the goal while driving, lower it.
    private static final double kS = 0.07;   // power needed just to start moving (static friction)
    private static final double MAX_POWER = 0.8;
    private static final double SWING_POWER = 0.5;   // power cap while the error is larger than SWING_ERR_DEG
    private static final double SWING_ERR_DEG = 25;
    private static final double TOLERANCE_RAD = Math.toRadians(0.5); // encoder is now about 0.03 deg per tick; inside this: stop pushing
    private static final double SPIN_FILTER = 0.35; // 0..1, higher = less smoothing of robot spin rate

    // Permanent aim correction. Aim perfectly with dpad trim, read the trim in telemetry,
    // and put that number here. Negative = turn RIGHT, positive = turn LEFT.
    private static final double AIM_OFFSET_RAD = Math.toRadians(0.0);

    private static final double AIM_TRIM_STEP_RAD = Math.toRadians(1.0);
    private static final double DRIVE_DEADZONE = 0.05;

    // ── State ────────────────────────────────────────────────────────────────
    private Follower follower;
    private DcMotorEx turret;

    private Alliance alliance = Alliance.RED;
    private int presetIndex = 0;
    private boolean aimOn = true;
    private double aimTrimRad = 0;

    private boolean lastB, lastX, lastRB, lastDL, lastDR;

    private final ElapsedTime dtTimer = new ElapsedTime();
    private double lastWanted = 0;
    private double lastTarget = 0; // the turret angle we were heading to last loop (see fitToRange)
    private double spinRate = 0; // how fast the wanted turret angle is changing, rad/s (name kept short)

    // telemetry
    private double dbgTargetRad, dbgErrRad, dbgPower;
    private double seenMinDeg = 0, seenMaxDeg = 0; // INIT limit finder

    // ── Helpers ──────────────────────────────────────────────────────────────
    private Pose resetPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    private double[] goal() {
        return (alliance == Alliance.RED ? RED_GOALS : BLUE_GOALS)[presetIndex];
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
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

    /** Turret angle (rad, in the turret's own frame) that points the shooter at the goal right now. */
    private double wantedTurretAngle(Pose pose) {
        double h = pose.heading();
        double px = pose.x() + PIVOT_X_IN * Math.cos(h) - PIVOT_Y_IN * Math.sin(h);
        double py = pose.y() + PIVOT_X_IN * Math.sin(h) + PIVOT_Y_IN * Math.cos(h);
        double[] g = goal();
        double fieldAngle = Math.atan2(g[1] - py, g[0] - px);
        return wrap(fieldAngle - h - TURRET_ZERO_OFFSET_RAD); // trim is added later so a trim press doesn't spike the rate
    }

    /**
     * Picks which of the equivalent angles (+-360 deg) the turret should go to.
     * It sticks with the SAME choice as last loop (nearest to prevTarget, so the target moves smoothly
     * as the robot moves) and only jumps to the other equivalent when the old choice would leave the
     * turret's range. That jump is the unwind. Because the choice is judged against prevTarget and NOT
     * against where the turret physically is, the target cannot flip back and forth while the turret is
     * still swinging around, and it keeps updating to the live goal bearing the whole way.
     * If no equivalent fits (the small gap in the range), it waits at the nearest limit.
     */
    private double fitToRange(double angle, double prevTarget) {
        double w = wrap(angle);
        double twoPi = 2 * Math.PI;
        double cont = w + Math.round((prevTarget - w) / twoPi) * twoPi; // continuous with last loop
        if (cont >= TURRET_MIN_RAD && cont <= TURRET_MAX_RAD) return cont;z

        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double a = w + k * twoPi;
            if (a < TURRET_MIN_RAD || a > TURRET_MAX_RAD) continue;
            if (Double.isNaN(best) || Math.abs(a - prevTarget) < Math.abs(best - prevTarget)) best = a;
        }
        if (!Double.isNaN(best)) return best; // unwind: only valid choice
        return Math.max(TURRET_MIN_RAD, Math.min(TURRET_MAX_RAD, cont)); // gap: wait at the limit
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(resetPose());

        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        // LIMIT FINDER: turn the turret by hand to each hard stop. Read the angle at each stop and
        // put them in TURRET_MIN_RAD / TURRET_MAX_RAD. Start it exactly facing back (angle 0).
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
        lastWanted = wantedTurretAngle(follower.pose());
        lastTarget = turretAngleRad();
        dtTimer.reset();
    }

    @Override
    public void loop() {
        // ── Buttons (rising edges) ───────────────────────────────────────
        boolean b = gamepad1.b, x = gamepad1.x, rb = gamepad1.right_bumper;
        boolean dl = gamepad1.dpad_left, dr = gamepad1.dpad_right;
        if (b && !lastB) presetIndex = 1 - presetIndex;
        if (x && !lastX) follower.setPose(resetPose());
        if (rb && !lastRB) aimOn = !aimOn;
        if (dl && !lastDL) aimTrimRad -= AIM_TRIM_STEP_RAD;
        if (dr && !lastDR) aimTrimRad += AIM_TRIM_STEP_RAD;
        lastB = b; lastX = x; lastRB = rb; lastDL = dl; lastDR = dr;

        // ── Drive ────────────────────────────────────────────────────────
        // Pedro: +lateral = left, +turn = counter-clockwise. Sticks are + to the right, so negate.
        follower.manual(deadzone(-gamepad1.left_stick_y),
                -deadzone(gamepad1.left_stick_x),
                -deadzone(gamepad1.right_stick_x));
        follower.update();

        // ── Robot spin rate (rad/s), filtered ────────────────────────────
        Pose pose = follower.pose();
        double dt = Math.max(dtTimer.seconds(), 1e-3);
        dtTimer.reset();
        // Rate the wanted angle is changing, from BOTH robot spin and robot translation.
        // Moving sideways close to the goal changes the bearing fast (rate = sideways speed / distance);
        // far away it changes slowly. Measuring the change of the wanted angle captures that automatically.
        double wanted = wantedTurretAngle(pose);
        double rawRate = wrap(wanted - lastWanted) / dt;
        lastWanted = wanted;
        spinRate += SPIN_FILTER * (rawRate - spinRate);

        // ── Turret ───────────────────────────────────────────────────────
        double current = turretAngleRad();
        double target = aimOn ? fitToRange(wanted + AIM_OFFSET_RAD + aimTrimRad, lastTarget) : 0;
        lastTarget = aimOn ? target : current; // aim turned back ON starts from where the turret really is
        double err = target - current;

        double power;
        if (Math.abs(err) > TOLERANCE_RAD) {
            power = kP * err - kD * turretSpeedRadPerSec() + Math.signum(err) * kS;
        } else {
            power = -kD * turretSpeedRadPerSec(); // just brake
        }
        if (aimOn) power += kV * spinRate; // follow how fast the goal bearing is changing (spin + driving)
        // Big swings (unwind / flick) are slower so the turret does not overshoot and then come back from the other side.
        double cap = Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? SWING_POWER : MAX_POWER;
        power = Math.max(-cap, Math.min(cap, power));
        turret.setPower(POWER_SIGN * power);

        dbgTargetRad = target;
        dbgErrRad = err;
        dbgPower = power;

        // ── Telemetry ────────────────────────────────────────────────────
        double[] g = goal();
        telemetry.addData("Alliance / preset", "%s #%d  goal=(%.0f, %.0f)", alliance, presetIndex, g[0], g[1]);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Aim", "%s trim=%.0f deg", aimOn ? "ON" : "OFF", Math.toDegrees(aimTrimRad));
        telemetry.addData("Turret", "now=%.1f target=%.1f err=%.2f deg %s",
                Math.toDegrees(current), Math.toDegrees(dbgTargetRad), Math.toDegrees(dbgErrRad),
                Math.abs(dbgErrRad) <= TOLERANCE_RAD ? "(ON TARGET)"
                        : (aimOn && Math.abs(dbgErrRad) > Math.toRadians(25) ? "(UNWINDING / far off)" : ""));
        // Field direction the shooter really points = robot heading + turret angle + zero offset (same math as wantedTurretAngle, reversed).
        // It should match "Goal bearing" when aimed.
        double[] gl = goal();
        double goalBearing = Math.atan2(gl[1] - pose.y(), gl[0] - pose.x());
        telemetry.addData("Turret heading", "%.1f deg (field)  goal bearing %.1f deg",
                Math.toDegrees(wrap(pose.heading() + current + TURRET_ZERO_OFFSET_RAD)), Math.toDegrees(goalBearing));
        telemetry.addData("Turret raw ticks", "%d  (%.1f ticks/deg assumed)",
                turret.getCurrentPosition(), TICKS_PER_RAD * Math.PI / 180.0);
        telemetry.addData("Goal rate / power", "%.0f deg/s  %.2f", Math.toDegrees(spinRate), dbgPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
    }
}
