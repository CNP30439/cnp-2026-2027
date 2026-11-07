package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// TeleMainTurret.java — TeleOp for the turret robot.
// Pedro manual drive + turret that keeps pointing at the goal (on/off switch)
// + shooter, intake, linkage, blocker, kicker, prism from the old TeleMain.
//
// REMOVED vs old TeleMain (all tied to the gamepad 1 R2 auto-drive, which the turret replaces):
//   R2 auto-drive, structure dodging, hold/aim-lock, auto-shoot. No Limelight for now.
//
// HARDWARE (names must match the Robot Configuration)
//   fl, fr, bl, br (Pedro Constants) | ls, rs shooter (rs REVERSED) | li, ri intake (ri REVERSED)
//   linkageleft, linkageright | blocker | kicker | prism | turret (ELC encoder on its encoder port)
//
// INIT (gamepad 1): dpad up = RED, dpad down = BLUE. Turret must be at its zero (facing back) at INIT.
//
// GAMEPAD 1 (driver)
//   left stick = drive/strafe, right stick X = rotate
//   B (press)            = switch preset 0 <-> 1 (changes goal + reset pose)
//   X (press)            = reset pose to the preset's start pose
//   right bumper (press) = TURRET AIM ON / OFF (OFF parks turret at zero)
//   dpad left / right    = trim aim -/+ 1 deg
//   hold Y / A           = jog turret + / - slowly (stop probe, overrides aim while held)
//
// GAMEPAD 2 (operator)
//   dpad up      = shooter on / off
//   dpad right   = shooter HIGH velocity     dpad left = shooter LOW velocity
//   left/right bumper = current mode's velocity -/+ 50 ticks/sec
//   A / B        = intake in / out (hold)
//   Y            = toggle flower up (0.7) / down (0.153)   [replaces the old linkage on X]
//   R2 (hold)    = SHOOT: intake IN, blocker OPEN, kicker fires after KICK_DELAY_MS.
//                  Release: kicker back to BASE, blocker CLOSED, delay restarts next press.
//   NOTE: R2 does not spin the shooter up. Turn it on with dpad up first.
//
// GAME RAMP: from PLAY the commanded shooter velocity creeps up by GAME_RAMP_EXTRA over GAME_RAMP_SECS.
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
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
    // Hard stops measured from the start (shooter facing back = 0), with 3 deg margin.
    private static final double TURRET_MIN_RAD = Math.toRadians(-87);
    private static final double TURRET_MAX_RAD = Math.toRadians(267);
    // Turret angle at INIT in the turret's own frame (0 = it starts facing back).
    private static final double TURRET_START_RAD = Math.toRadians(0.0);
    // Direction the shooter points relative to robot forward when the turret is at zero. PI = backward.
    private static final double TURRET_ZERO_OFFSET_RAD = Math.PI;

    // Turret pivot relative to robot center, inches. x = forward, y = left.
    private static final double PIVOT_X_IN = 0.0;
    private static final double PIVOT_Y_IN = 0.0;

    private static final double kP = 3.0;    // power per rad of error
    private static final double kD = 0.08;   // power per rad/s of turret speed
    private static final double kV = 0.12;   // power per rad/s of wanted-angle change (feedforward)
    private static final double kS = 0.07;   // static friction power
    private static final double MAX_POWER = 0.8;
    private static final double SWING_POWER = 0.5;   // cap while error > SWING_ERR_DEG
    private static final double SWING_ERR_DEG = 25;
    private static final double TOLERANCE_RAD = Math.toRadians(0.5);
    private static final double SPIN_FILTER = 0.35;  // 0..1, higher = less smoothing

    // Permanent aim correction (negative = right, positive = left). Find it with the dpad trim.
    private static final double AIM_OFFSET_RAD = Math.toRadians(0.0);
    private static final double AIM_TRIM_STEP_RAD = Math.toRadians(1.0);
    private static final double JOG_POWER = 0.2;

    // ── Turret state ─────────────────────────────────────────────────────────
    private DcMotorEx turret;
    private boolean aimOn = true;
    private double aimTrimRad = 0;
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
    private double shooterHighVelocity = 1000;
    private double shooterLowVelocity = 1000;
    private static final double VELOCITY_STEP = 50;
    private static final double VELOCITY_MIN = 0;
    private boolean shooterOn = false;
    private boolean shooterHigh = false;

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
    private boolean lastB, lastX1, lastRB1, lastDL1, lastDR1;
    private boolean lastDpadUp, lastDpadLeft, lastDpadRight;
    private boolean lastY2, lastLB2, lastRB2;

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
    // init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        alliance = Alliance.RED;
        presetIndex = 0;
        follower.setPose(resetPose());

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
        boolean dl = gamepad1.dpad_left, dr = gamepad1.dpad_right;
        if (b && !lastB) presetIndex = 1 - presetIndex;
        if (x1 && !lastX1) follower.setPose(resetPose());
        if (rb1 && !lastRB1) aimOn = !aimOn;
        if (dl && !lastDL1) aimTrimRad -= AIM_TRIM_STEP_RAD;
        if (dr && !lastDR1) aimTrimRad += AIM_TRIM_STEP_RAD;
        lastB = b; lastX1 = x1; lastRB1 = rb1; lastDL1 = dl; lastDR1 = dr;

        // ── Drive ────────────────────────────────────────────────────────
        // Pedro: +lateral = left, +turn = CCW. Sticks are + to the right, so negate.
        follower.manual(deadzone(-gamepad1.left_stick_y),
                -deadzone(gamepad1.left_stick_x),
                -deadzone(gamepad1.right_stick_x));
        follower.update();

        // ── Turret ───────────────────────────────────────────────────────
        Pose pose = follower.pose();
        double dt = Math.max(dtTimer.seconds(), 1e-3);
        dtTimer.reset();
        // Rate the wanted angle changes (robot spin + translation), filtered.
        double wanted = wantedTurretAngle(pose);
        double rawRate = wrap(wanted - lastWanted) / dt;
        lastWanted = wanted;
        spinRate += SPIN_FILTER * (rawRate - spinRate);

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
        if (aimOn) power += kV * spinRate; // follows how fast the goal bearing changes (robot turning / driving)
        double cap = Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? SWING_POWER : MAX_POWER;
        power = Math.max(-cap, Math.min(cap, power));

        // Stop probe: hold Y (+, CCW) or A (-, CW) to creep into a hard stop.
        boolean jog = gamepad1.y ^ gamepad1.a;
        if (jog) {
            power = gamepad1.y ? JOG_POWER : -JOG_POWER;
            lastTarget = current;
        }
        turret.setPower(POWER_SIGN * power);

        // ── Shooter mode (gamepad 2 dpad) ────────────────────────────────
        boolean dUp = gamepad2.dpad_up, dLeft = gamepad2.dpad_left, dRight = gamepad2.dpad_right;
        if (dUp && !lastDpadUp) shooterOn = !shooterOn;
        if (dRight && !lastDpadRight) shooterHigh = true;
        if (dLeft && !lastDpadLeft) shooterHigh = false;
        lastDpadUp = dUp; lastDpadRight = dRight; lastDpadLeft = dLeft;

        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD;

        // ── Flywheel speed trim ──────────────────────────────────────────
        boolean lb2 = gamepad2.left_bumper, rb2 = gamepad2.right_bumper;
        if (rb2 && !lastRB2) {
            if (shooterHigh) shooterHighVelocity += VELOCITY_STEP;
            else shooterLowVelocity += VELOCITY_STEP;
        }
        if (lb2 && !lastLB2) {
            if (shooterHigh) shooterHighVelocity = Math.max(VELOCITY_MIN, shooterHighVelocity - VELOCITY_STEP);
            else shooterLowVelocity = Math.max(VELOCITY_MIN, shooterLowVelocity - VELOCITY_STEP);
        }
        lastLB2 = lb2; lastRB2 = rb2;

        double shooterVelocity;
        String shooterMode;
        if (!shooterOn) {
            shooterVelocity = 0;
            shooterMode = "OFF";
        } else if (shooterHigh) {
            shooterVelocity = shooterHighVelocity;
            shooterMode = "HIGH";
        } else {
            shooterVelocity = shooterLowVelocity;
            shooterMode = "LOW";
        }

        double gameRamp = GAME_RAMP_EXTRA * Math.min(1.0, gameTimer.seconds() / GAME_RAMP_SECS);
        double commandedVelocity = shooterOn ? (shooterVelocity + gameRamp) : 0;
        leftShooter.setVelocity(commandedVelocity);

        // ── SHOOT (gamepad 2 R2, held) ───────────────────────────────────
        if (shooting && !lastShooting) kickTimer.reset();
        lastShooting = shooting;
        boolean kickReady = shooting && kickTimer.milliseconds() >= KICK_DELAY_MS;

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
        telemetry.addData("Shooter HIGH/LOW targets", "%.0f / %.0f", shooterHighVelocity, shooterLowVelocity);
        telemetry.addData("Game ramp", "+%.1f ticks/sec (%.0f s elapsed)", gameRamp, gameTimer.seconds());
        telemetry.addData("Shooting (G2 R2)", shooting ? "YES" : "no");
        if (shooting) {
            telemetry.addData("Kick delay", "%.0f / %d ms %s",
                    Math.min(kickTimer.milliseconds(), KICK_DELAY_MS), KICK_DELAY_MS,
                    kickReady ? "(FIRING)" : "(waiting)");
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
        telemetry.addData("TURRET AIM", "%s trim=%.0f deg", aimOn ? "ON" : "OFF", Math.toDegrees(aimTrimRad));
        telemetry.addData("Turret", "now=%.1f target=%.1f err=%.2f deg %s",
                Math.toDegrees(current), Math.toDegrees(target), Math.toDegrees(err),
                !aimOn ? "(parked)" : Math.abs(err) <= TOLERANCE_RAD ? "(ON TARGET)"
                        : (Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? "(far off)" : ""));
        double goalBearing = Math.atan2(g[1] - pose.y(), g[0] - pose.x());
        telemetry.addData("Turret heading", "%.1f deg (field)  goal bearing %.1f deg",
                Math.toDegrees(wrap(pose.heading() + current + TURRET_ZERO_OFFSET_RAD)), Math.toDegrees(goalBearing));
        telemetry.addData("Turret raw ticks", "%d", turret.getCurrentPosition());
        telemetry.addData("Goal rate / power", "%.0f deg/s  %.2f", Math.toDegrees(spinRate), power);
        telemetry.addData("Stop probe", "hold Y = turret + (CCW), hold A = turret - (CW) %s", jog ? "<< JOGGING" : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
        leftShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        setFlowerPosition(FLOWER_DOWN);
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);
    }
}
