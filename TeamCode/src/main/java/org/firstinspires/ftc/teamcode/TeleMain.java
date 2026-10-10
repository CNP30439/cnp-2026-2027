package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  TeleMain.java  —  TeleOp. Driving goes through Pedro Pathing (position hold
//                    when idle + hold-R2 auto-drive to a fixed pose). Shooter,
//                    intake, servos and sensor are plain hardware.
//
//  HARDWARE (names must match the Robot Configuration on the Driver Hub)
//    4x drive motors    — fl, fr, bl, br    (mecanum)
//    2x shooter motors  — ls, rs            (rs REVERSED so both spin the same way)
//    2x intake motors   — li, ri            (ri REVERSED so both pull inward)
//    2x linkage servos  — linkageleft, linkageright  (always same position)
//    1x blocker servo   — blocker           (toggles OPEN / CLOSED)
//    1x hood servo      — hood              (driven by R2 — see HOOD section below)
//    1x kicker servo    — kicker            (BASE / KICK — pushes last ball up)
//    1x distance sensor — distance          (telemetry only — no longer drives the kicker)
//    1x Prism light     — prism             (GoBilda Prism driver, status animations)
//
//  CONTROLS
//    Gamepad 1
//      left stick        — drive / strafe
//      right stick X     — rotate
//      R2 (hold)         — auto-drive to IDEAL_POSE (turns while it moves).
//                          Release R2 or move a stick to cancel.
//      X                 — reset pose to ORIGIN (robot on the reset spot,
//                          facing the same way). Ignored while aligning.
//      After R2 arrives  — Pedro holds the target position until you move a
//                          stick. Otherwise there is NO position hold.
//
//    Gamepad 2
//      dpad up           — toggle shooter on / off
//      dpad right        — shooter HIGH velocity
//      dpad left         — shooter LOW velocity
//      dpad down         — set hood to HOOD_START (manual override; harmless —
//                          R2 logic below sets the same value every loop anyway
//                          whenever you're not shooting)
//      left bumper       — decrease current mode's shooter velocity by 100 ticks/sec
//      right bumper      — increase current mode's shooter velocity by 100 ticks/sec
//      A                 — intake in  (hold)
//      B                 — intake out (hold)
//      X                 — toggle linkage up / down
//      Y                 — toggle blocker open / closed
//      R2 (right trigger)— SHOOT (hold): intake runs IN and blocker OPENS
//                          immediately. After KICK_DELAY_MS has elapsed from
//                          when R2 was first pressed, the kicker fires and
//                          stays at KICK for the rest of the hold. Release R2
//                          and the kicker snaps straight back to BASE; the
//                          delay resets fresh on every new R2 press.
//                          Holding R2 also ramps the shooter up by +BOOST_MAX
//                          ticks/sec over BOOST_RAMP_SECS (0.2s); releasing
//                          R2 ramps that boost back down over the same time.
//                          R2 ALSO drives the hood directly (no separate
//                          button) — see HOOD below.
//
//  HOOD (driven entirely by R2 — no separate button)
//    While R2 is held (shooting == true): hood snaps to HOOD_LOWEST.
//    The instant R2 is released:          hood snaps back to HOOD_START.
//    TODO: HOOD_LOWEST below is a placeholder — set it to your real tuned
//    value once you've found it on the bench.
//
//  SHOOT SEQUENCE (while R2 held)
//    1. Blocker forced OPEN, intake forced IN — happens the instant R2 is pressed.
//    2. Kicker waits at BASE for KICK_DELAY_MS from the moment R2 was pressed,
//       then moves to KICK and stays there for the rest of the hold.
//    3. On release of R2, kicker snaps back to BASE and the delay resets fresh
//       for the next R2 press.
//    NOTE: R2 does not turn the shooter on by itself — spin it up with dpad up
//    first so the flywheels are at speed when balls arrive.
//
//  SHOOTER VELOCITY CONTROL
//    Closed-loop velocity (ticks/sec) using PIDF gains from FlywheelTuner.
//    Left/right bumper nudge the current HIGH or LOW target by VELOCITY_STEP.
//    While R2 is held, shooter also gets a boost ramping up to +BOOST_MAX
//    ticks/sec over BOOST_RAMP_SECS (0.2s), ramping back down on release.
//
//  VOLTAGE / CURRENT MONITORING
//    Voltage is per-hub only; current is per DcMotorEx only.
//    Servos, distance sensor, and Prism have no SDK voltage/current reading.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@TeleOp(name = "TeleMain", group = "TeleOp")
public class TeleMain extends OpMode {

    // ── Drive ────────────────────────────────────────────────────────────────
    // Pedro's follower owns the four drive motors (names/directions are set in
    // pedro/Constants.java), so there are no drive motor objects in this file.
    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Auto-align to a fixed pose (gamepad 1, R2) ───────────────────────────
    private enum AlignState { MANUAL, ALIGNING, ARRIVED }

    private static final PoseFactory p = PoseFactory.degrees();

    // Reset / starting spot.
    public static final Pose ORIGIN = p.of(8, 9, 90);

    // Auto-drive target.
    public static final Pose IDEAL_POSE = p.of(56, 14, -90);

    // Stick movement past this cancels an in-progress alignment.
    private static final double STICK_ABORT_THRESHOLD = 0.15;

    private AlignState alignState    = AlignState.MANUAL;
    private boolean    alignWasHeld  = false;   // gamepad1 R2 last tick
    private boolean    resetWasHeld  = false;   // gamepad1 X last tick
    // True once the follower has actually become busy after scheduling; stops
    // the first tick from wrongly jumping to ARRIVED.
    private boolean    alignStarted  = false;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;

    private static final double SHOOTER_P       = 1.5;
    private static final double SHOOTER_F_LEFT  = 15;
    private static final double SHOOTER_F_RIGHT = 12.77;
    private GoBildaPrismDriver prism;

    private double shooterHighVelocity = 2250;
    private double shooterLowVelocity  = 1500;
    private static final double VELOCITY_STEP = 100;
    private static final double VELOCITY_MIN  = 0;

    private boolean shooterOn   = false;
    private boolean shooterHigh = false;

    // Boost ramps from 0 -> BOOST_MAX over BOOST_RAMP_SECS (0.2s), and back
    // down over the same time once R2 is released.
    private double shooterBoost = 0;
    private static final double BOOST_MAX       = 0;
    private static final double BOOST_RAMP_SECS = 0;
    private static final double BOOST_RATE      = BOOST_MAX / BOOST_RAMP_SECS; // 2750 ticks/sec²
    private final ElapsedTime boostTimer = new ElapsedTime();

    // ── Intake ───────────────────────────────────────────────────────────────
    private DcMotorEx leftIntake, rightIntake;
    private static final double INTAKE_POWER = 1.0;

    // ── Linkage ──────────────────────────────────────────────────────────────
    private Servo linkageLeft, linkageRight;
    private static final double LINKAGE_DOWN = 0;
    private static final double LINKAGE_UP   = .5;
    private boolean linkageUp = false;
    private int currentArtboardState = -1;

    // ── Blocker ──────────────────────────────────────────────────────────────
    private Servo blocker;
    private static final double BLOCKER_CLOSED = 1.0;
    private static final double BLOCKER_OPEN   = 0.6;
    private boolean blockerOpen = false;

    // ── Hood ─────────────────────────────────────────────────────────────────
    private Servo hood;
    private static final double HOOD_START  = 0.5;
    private static final double HOOD_LOWEST = 0.35; // TODO — tune on the bench

    // ── Kicker ───────────────────────────────────────────────────────────────
    private Servo kicker;
    private static final double KICKER_BASE    = 0.8;
    private static final double KICKER_KICK    = 0.5;
    private static final long   KICK_DELAY_MS  = 500; // ms after R2 press before kicker fires

    private final ElapsedTime kickTimer  = new ElapsedTime();
    private boolean lastShooting = false;

    // ── Distance sensor (telemetry only) ─────────────────────────────────────
    private DistanceSensor ballSensor;
    private static final double BALL_PRESENT_INCHES = 2.0;

    // ── Voltage monitoring ───────────────────────────────────────────────────
    private static final double VOLTAGE_GOOD_MIN = 12.0;
    private static final double VOLTAGE_OK_MIN   = 11.0;
    private static final double VOLTAGE_LOW_MIN  =  9.5;

    // ── Shoot trigger ────────────────────────────────────────────────────────
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    private boolean lastDpadUp      = false;
    private boolean lastDpadLeft    = false;
    private boolean lastDpadRight   = false;
    private boolean lastDpadDown    = false;
    private boolean lastX           = false;
    private boolean lastY           = false;
    private boolean lastLeftBumper  = false;
    private boolean lastRightBumper = false;

    // ═════════════════════════════════════════════════════════════════════════
    //  init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        // Drive is handled by Pedro (motors fl/fr/bl/br are configured in Constants).
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(ORIGIN);
        alignState   = AlignState.MANUAL;
        alignWasHeld = false;
        resetWasHeld = false;
        alignStarted = false;

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        currentArtboardState = 0;

        leftShooter  = hardwareMap.get(DcMotorEx.class, "ls");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rs");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT));
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_RIGHT));

        leftIntake  = hardwareMap.get(DcMotorEx.class, "li");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linkageLeft  = hardwareMap.get(Servo.class, "linkageleft");
        linkageRight = hardwareMap.get(Servo.class, "linkageright");
        linkageRight.setDirection(Servo.Direction.REVERSE);   // <-- add this
        linkageUp = false;
        setLinkagePosition(LINKAGE_DOWN);

        blocker = hardwareMap.get(Servo.class, "blocker");
        blockerOpen = false;
        blocker.setPosition(BLOCKER_CLOSED);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(HOOD_START);

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_BASE);

        ballSensor = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addLine("Initialized — press PLAY");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        currentArtboardState = 0;

        // ── Drive ────────────────────────────────────────────────────────────
        double stickY  = -gamepad1.left_stick_y;
        double stickX  =  gamepad1.left_stick_x;
        double stickRx =  gamepad1.right_stick_x;

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        // Gamepad 1: X resets pose, R2 starts/cancels auto-align.
        updateAlign(Math.max(Math.abs(stickY), Math.max(Math.abs(stickX), Math.abs(stickRx))));

        if (driverHasControl()) {
            // Pedro: +lateral = left, +turn = counterclockwise. Sticks are
            // positive to the right, so both are negated.
            double lateral = -stickX;
            double turn    = -stickRx;
            // Plain manual drive: no position hold, no extra stick cutoff
            // beyond DRIVE_DEADZONE, so slow strafing works.
            follower.manual(stickY, lateral, turn);
        }

        follower.update();
        Scheduler.execute();

        // ── Shooter mode (dpad) ───────────────────────────────────────────────
        boolean dUp    = gamepad2.dpad_up;
        boolean dLeft  = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;
        boolean dDown  = gamepad2.dpad_down;

        if (dUp    && !lastDpadUp)    shooterOn   = !shooterOn;
        if (dRight && !lastDpadRight) shooterHigh = true;
        if (dLeft  && !lastDpadLeft)  shooterHigh = false;
        if (dDown  && !lastDpadDown)  hood.setPosition(HOOD_START);

        lastDpadUp    = dUp;
        lastDpadRight = dRight;
        lastDpadLeft  = dLeft;
        lastDpadDown  = dDown;

        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD;

        // ── Hood ─────────────────────────────────────────────────────────────
        hood.setPosition(shooting ? HOOD_LOWEST : HOOD_START);

        // ── Flywheel speed trim ───────────────────────────────────────────────
        boolean curLeftBumper  = gamepad2.left_bumper;
        boolean curRightBumper = gamepad2.right_bumper;

        if (curRightBumper && !lastRightBumper) {
            if (shooterHigh) shooterHighVelocity += VELOCITY_STEP;
            else              shooterLowVelocity  += VELOCITY_STEP;
        }
        if (curLeftBumper && !lastLeftBumper) {
            if (shooterHigh) shooterHighVelocity = Math.max(VELOCITY_MIN, shooterHighVelocity - VELOCITY_STEP);
            else              shooterLowVelocity  = Math.max(VELOCITY_MIN, shooterLowVelocity  - VELOCITY_STEP);
        }
        lastLeftBumper  = curLeftBumper;
        lastRightBumper = curRightBumper;

        double shooterVelocity;
        String shooterMode;
        if (!shooterOn) {
            shooterVelocity = 0;
            shooterMode     = "OFF";
        } else if (shooterHigh) {
            shooterVelocity = shooterHighVelocity;
            shooterMode     = "HIGH";
        } else {
            shooterVelocity = shooterLowVelocity;
            shooterMode     = "LOW";
        }

        // Boost: 0 -> BOOST_MAX in 0.2s while R2 held, back to 0 in 0.2s on release.
        double boostDt = boostTimer.seconds();
        boostTimer.reset();
        double boostTarget = shooting ? BOOST_MAX : 0;
        if (shooterBoost < boostTarget) {
            shooterBoost = Math.min(boostTarget, shooterBoost + BOOST_RATE * boostDt);
        } else if (shooterBoost > boostTarget) {
            shooterBoost = Math.max(boostTarget, shooterBoost - BOOST_RATE * boostDt);
        }

        double commandedVelocity = shooterOn ? (shooterVelocity + shooterBoost) : 0;
        leftShooter.setVelocity(commandedVelocity);
        rightShooter.setVelocity(commandedVelocity);

        // ── Blocker toggle (Y) ────────────────────────────────────────────────
        boolean curY = gamepad2.y;
        if (curY && !lastY) blockerOpen = !blockerOpen;
        lastY = curY;

        // ── Ball sensor (telemetry only) ──────────────────────────────────────
        double ballDist    = ballSensor.getDistance(DistanceUnit.INCH);
        boolean ballPresent = !Double.isNaN(ballDist) && ballDist <= BALL_PRESENT_INCHES;

        // ── SHOOT (R2, held) ──────────────────────────────────────────────────
        // Rising edge: reset kick timer so the delay counts from this press.
        if (shooting && !lastShooting) {
            kickTimer.reset();
        }
        lastShooting = shooting;

        // Kicker fires once KICK_DELAY_MS has elapsed from the R2 press.
        boolean kickReady = shooting && kickTimer.milliseconds() >= KICK_DELAY_MS;

        double intakePower;
        String intakeMode;

        if (shooting) {
            intakePower = INTAKE_POWER;
            intakeMode  = "SHOOT-FEED";
            blocker.setPosition(BLOCKER_OPEN);
            kicker.setPosition(kickReady ? KICKER_KICK : KICKER_BASE);
        } else {
            if (gamepad2.a) {
                intakePower = INTAKE_POWER;
                intakeMode  = "IN";
            } else if (gamepad2.b) {
                intakePower = -INTAKE_POWER;
                intakeMode  = "OUT";
            } else {
                intakePower = 0;
                intakeMode  = "STOPPED";
            }
            blocker.setPosition(blockerOpen ? BLOCKER_OPEN : BLOCKER_CLOSED);
            kicker.setPosition(KICKER_BASE);
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // ── Linkage (X toggle) ────────────────────────────────────────────────
        boolean curX = gamepad2.x;
        if (curX && !lastX) {
            linkageUp = !linkageUp;
            setLinkagePosition(linkageUp ? LINKAGE_UP : LINKAGE_DOWN);
        }
        lastX = curX;

        // ── Telemetry ────────────────────────────────────────────────────────
        boolean blockerIsOpen = shooting || blockerOpen;
        telemetry.addData("Shooter", "%s target=%.0f boost=%.0f commanded=%.0f actual(L/R)=%.0f/%.0f",
                shooterMode, shooterVelocity, shooterBoost, commandedVelocity,
                leftShooter.getVelocity(), rightShooter.getVelocity());
        telemetry.addData("Shooter HIGH/LOW targets", "%.0f / %.0f", shooterHighVelocity, shooterLowVelocity);
        telemetry.addData("Shooting (R2)", shooting ? "YES" : "no");
        if (shooting) {
            telemetry.addData("Kick delay", "%.0f / %d ms  %s",
                    Math.min(kickTimer.milliseconds(), KICK_DELAY_MS), KICK_DELAY_MS,
                    kickReady ? "(FIRING)" : "(waiting)");
        }
        telemetry.addData("Intake",  "%s", intakeMode);
        telemetry.addData("Blocker", blockerIsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker",  kickReady ? "KICK" : "BASE");
        telemetry.addData("Ball dist (in)", "%.1f  %s", ballDist, ballPresent ? "BALL" : "empty");
        telemetry.addData("Linkage", linkageUp ? "UP" : "DOWN");
        telemetry.addData("Hood",    "%.2f  (LOWEST=%.2f START=%.2f)", hood.getPosition(), HOOD_LOWEST, HOOD_START);
        telemetry.addLine("---");
        Pose pose = follower.pose();
        telemetry.addData("Align", "%s  busy=%b  mode=%s", alignState, follower.isBusy(), follower.mode());
        telemetry.addData("Pose", "x=%.1f  y=%.1f  heading=%.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to target", "%.1f in", distanceToTarget());


        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  stop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        Scheduler.reset();
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        setLinkagePosition(LINKAGE_DOWN);
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);
        hood.setPosition(HOOD_START);
    }

    private void setLinkagePosition(double position) {
        linkageLeft.setPosition(position);
        linkageRight.setPosition(position);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Auto-align to IDEAL_POSE (gamepad 1)
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Gamepad 1 X = reset pose to ORIGIN. Gamepad 1 R2 (hold) = auto-drive to
     * IDEAL_POSE. Only one thing owns the drivetrain at a time:
     *   MANUAL   — driver (plain stick drive, no position hold)
     *   ALIGNING — Ivy is driving the path
     *   ARRIVED  — path done; Pedro holds the target until a stick moves.
     *              Press R2 again to drive to the target again.
     */
    private void updateAlign(double stickMag) {
        // X: reset pose once per press, only while the driver has control.
        boolean xHeld = gamepad1.x;
        if (xHeld && !resetWasHeld && driverHasControl()) {
            follower.setPose(ORIGIN);
        }
        resetWasHeld = xHeld;

        boolean alignHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean rising    = alignHeld && !alignWasHeld;
        alignWasHeld = alignHeld;

        switch (alignState) {

            case MANUAL:
                // Only a fresh press starts an alignment.
                if (rising) {
                    schedule(follow(follower, pathToIdeal()));
                    alignStarted = false;
                    alignState = AlignState.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!alignHeld || stickMag > STICK_ABORT_THRESHOLD) {
                    // Driver let go or grabbed the sticks.
                    Scheduler.reset();   // clears ALL scheduled Ivy commands
                    alignState = AlignState.MANUAL;
                } else if (follower.isBusy()) {
                    alignStarted = true;     // path is genuinely running
                } else if (alignStarted) {
                    // Reached the target via R2: now (and only now) hold it.
                    schedule(hold(follower, IDEAL_POSE));
                    alignState = AlignState.ARRIVED;
                }
                // !started and !busy: command queued, not started yet. Wait.
                break;

            case ARRIVED:
                // Pedro is holding the target. Any stick movement hands control
                // back to the driver (stickMag is already past DRIVE_DEADZONE).
                if (stickMag > 0) {
                    Scheduler.reset();   // stops the hold
                    alignState = AlignState.MANUAL;
                } else if (rising) {
                    // R2 pressed again: drive to the target again.
                    Scheduler.reset();
                    schedule(follow(follower, pathToIdeal()));
                    alignStarted = false;
                    alignState = AlignState.ALIGNING;
                }
                break;
        }
    }

    /** True only in MANUAL. While ALIGNING Ivy drives; while ARRIVED Pedro holds the target. */
    private boolean driverHasControl() {
        return alignState == AlignState.MANUAL;
    }

    /**
     * Straight line from the robot's current pose to IDEAL_POSE. .linear()
     * turns the heading steadily while it drives, so the robot rotates to the
     * target heading as it moves.
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