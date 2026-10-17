package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
//  AutoShootDodge.java  —  TeleOp. Driving goes through Pedro Pathing (position hold
//                    when idle + hold-R2 auto-drive to a fixed pose). Shooter,
//                    intake, servos and sensor are plain hardware.
//
//  HARDWARE (names must match the Robot Configuration on the Driver Hub)
//    4x drive motors    — fl, fr, bl, br    (mecanum)
//    2x shooter motors  — ls, rs            (rs REVERSED so both spin the same way)
//    2x intake motors   — li, ri            (li REVERSED so both pull inward)
//    2x linkage servos  — linkageleft, linkageright  (always same position)
//    1x blocker servo   — blocker           (toggles OPEN / CLOSED)
//    1x hood servo      — hood              (driven by R2 — see HOOD section below)
//    1x kicker servo    — kicker            (BASE / KICK — pushes last ball up)
//    1x distance sensor — distance          (telemetry only — no longer drives the kicker)
//    1x Prism light     — prism             (GoBilda Prism driver, status animations)
//
//  ALLIANCE / PRESET SELECTION
//    During INIT (before PLAY), gamepad 1:
//      dpad up           — choose RED side
//      dpad down         — choose BLUE side
//    The choice shows in telemetry. Default is RED.
//    Each side has TWO presets (0 and 1). Each preset has its own
//    shooting pose (R2 auto-drive target) and its own reset pose (X button).
//    Preset 0 is the one you start on. Gamepad 1 B (press) toggles 0 <-> 1.
//    Fill in the coordinates in the RED_* / BLUE_* arrays below.
//
//  CONTROLS
//    Gamepad 1
//      left stick        — drive / strafe
//      right stick X     — rotate
//      B (press)         — switch preset (changes BOTH the shooting pose and
//                          the reset pose for the chosen alliance).
//      R2 (hold)         — auto-drive to the current shooting pose (turns
//                          while it moves). Release R2 to cancel / take back control.
//      X                 — reset pose to the current reset pose (robot on the
//                          reset spot, facing the same way). Ignored while aligning.
//      After R2 arrives  — Pedro holds the target position while R2 is held.
//                          Release R2 and you drive normally (no position hold).
//                          AUTO-SHOOT: once it arrives, the robot also starts the
//                          same shoot sequence as gamepad 2 R2 (below) for as long
//                          as you keep holding R2. The shooter must already be on
//                          (gamepad 2 dpad up).
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
//                          when R2 was first pressed, the kicker goes UP ONCE,
//                          and as soon as it has had time to reach the KICK
//                          position (KICK_TRAVEL_MS) it goes straight back DOWN
//                          to BASE. It does NOT stay up while R2 is held.
//                          To kick again: release R2 and press it again (the
//                          delay and the kick reset fresh on every new press).
//                          Holding R2 also ramps the shooter up by +BOOST_MAX
//                          ticks/sec over BOOST_RAMP_SECS; releasing
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
//    2. Kicker waits at BASE for KICK_DELAY_MS from the moment R2 was pressed.
//    3. Kicker moves to KICK. Once KICK_TRAVEL_MS has passed (time for the
//       servo to reach the position) it returns to BASE and stays there.
//    4. Release R2 and press again to run the sequence again.
//    NOTE: R2 does not turn the shooter on by itself — spin it up with dpad up
//    first so the flywheels are at speed when balls arrive.
//
//  SHOOTER VELOCITY CONTROL
//    Closed-loop velocity (ticks/sec) using PIDF gains from FlywheelTuner.
//    Left/right bumper nudge the current HIGH or LOW target by VELOCITY_STEP.
//    While R2 is held, shooter also gets a boost ramping up to +BOOST_MAX
//    ticks/sec over BOOST_RAMP_SECS, ramping back down on release.
//    STRUCTURE DODGING: the R2 auto-drive avoids the two rectangles in
//    KEEP_OUT_ZONES, staying ROBOT_MARGIN (pose is the robot center) away
//    from their edges. If the straight line is clear it drives straight; if not
//    it follows one continuous rounded path around the structures (or between them), checked
//    against the margin before it is used. Rectangles left as {0,0,0,0} are
//    ignored until you fill in the coordinates.
//    GAME RAMP: from PLAY, the commanded velocity also creeps up linearly by
//    GAME_RAMP_EXTRA (25) over GAME_RAMP_SECS (120 s), so 1250 -> 1275 across
//    the full 2 minutes, then holds at +25.
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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@TeleOp(name = "Autoshotdodge😁", group = "TeleOp")
public class AutoShootDodge extends OpMode {

    // ── Drive ────────────────────────────────────────────────────────────────
    // Pedro's follower owns the four drive motors (names/directions are set in
    // pedro/Constants.java), so there are no drive motor objects in this file.
    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Auto-align to a fixed pose (gamepad 1, R2) ───────────────────────────
    private enum AlignState { MANUAL, ALIGNING, ARRIVED }

    private static final PoseFactory p = PoseFactory.degrees();

    // ── Alliance + preset poses ──────────────────────────────────────────────
    // Chosen in INIT with gamepad 1 dpad up (RED) / dpad down (BLUE).
    private enum Alliance { RED, BLUE }

    // Each array has two presets: [0] = default, [1] = alternate (gamepad 1 B).
    // SHOOT = R2 auto-drive target.  RESET = where X resets the pose to.
    public static final Pose[] RED_SHOOT_POSES = {
            p.of(56, 14, -90),   // RED preset 0 shooting pose
            p.of(56, 130, 90)    // RED preset 1 shooting pose
    };
    public static final Pose[] RED_RESET_POSES = {
            p.of(7.25, 8, -90),     // RED preset 0 reset pose
            p.of(7.25, 136, 90)     // RED preset 1 reset pose
    };

    public static final Pose[] BLUE_SHOOT_POSES = {
            p.of(83, 14, -90),   // BLUE preset 0 shooting pose
            p.of(83, 130, 90)    // BLUE preset 1 shooting pose
    };
    public static final Pose[] BLUE_RESET_POSES = {
            p.of(135.25, 8, -90),   // BLUE preset 0 reset pose
            p.of(135.25, 136, 90)   // BLUE preset 1 reset pose
    };

    private Alliance alliance    = Alliance.RED;
    private int      presetIndex = 0;       // 0 or 1, toggled by gamepad 1 B
    private boolean  lastPresetBtn = false;

    private AlignState alignState    = AlignState.MANUAL;
    private boolean    alignWasHeld  = false;   // gamepad1 R2 last tick
    private boolean    resetWasHeld  = false;   // gamepad1 X last tick
    // True once the follower has actually become busy after scheduling; stops
    // the first tick from wrongly jumping to ARRIVED.
    private boolean    alignStarted  = false;
    // The pose the current align is driving to / holding. Captured when R2 is
    // pressed so pressing B mid-align can't make the path and hold disagree.
    private Pose       alignTarget   = RED_SHOOT_POSES[0];

    // ── Structure dodging ────────────────────────────────────────────────────
    // The field has TWO structures to avoid. Each is a rectangle in field inches:
    //     { minX, minY, maxX, maxY }
    // The auto-drive checks its straight line to the shooting pose against both.
    // If the line would clip one, it plans a detour around the (padded) corners
    // instead. A rectangle left as {0, 0, 0, 0} is ignored, so the drive is
    // unchanged until you fill them in.
    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91},   // STRUCTURE 1  {minX, minY, maxX, maxY}
            {91, 50, 96, 91}    // STRUCTURE 2  {minX, minY, maxX, maxY}
    };

    // The robot pose is its CENTER, so it must stay at least this far from the
    // edge of any structure.
    private static final double ROBOT_MARGIN = 9.0;

    // When the straight line is blocked the robot follows one continuous, smooth
    // path (Paths.path of lines + curves): straight runs joined by rounded corners, so it
    // sweeps around a structure (or squeezes between the two) without stopping.
    // CORNER_RADIUS is how far before/after each corner the rounding starts. The
    // finished path is sampled and checked against ROBOT_MARGIN; if a rounded
    // corner cuts too close, the corner is pushed out by PAD_STEP and re-checked.
    private static final double CORNER_RADIUS = 10.0;
    private static final int    CURVE_SAMPLES = 20;     // per segment
    private static final double PAD_STEP      = 1.5;
    private static final int    PAD_TRIES     = 10;
    private static final double FIELD_MIN    = 0;
    private static final double FIELD_MAX    = 144;

    // How many detour guide points the current auto-drive curve bends around (0 = straight).
    private int routeCorners = 0;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;

    private static final double SHOOTER_P       = 1.5;
    private static final double SHOOTER_F_LEFT  = 18.3237;
    private static final double SHOOTER_F_RIGHT = 18.3237;
    private GoBildaPrismDriver prism;

    private double shooterHighVelocity = 1200;
    private double shooterLowVelocity  = 1200;
    private static final double VELOCITY_STEP = 50;
    private static final double VELOCITY_MIN  = 0;

    private boolean shooterOn   = false;
    private boolean shooterHigh = false;

    // Boost ramps from 0 -> BOOST_MAX over BOOST_RAMP_SECS, and back
    // down over the same time once R2 is released.
    private double shooterBoost = 0;
    private static final double BOOST_MAX       = 0;
    private static final double BOOST_RAMP_SECS = 0;
    private static final double BOOST_RATE      = BOOST_MAX / BOOST_RAMP_SECS;
    private final ElapsedTime boostTimer = new ElapsedTime();

    // Game-long ramp: the shooter target creeps up linearly from the HIGH/LOW
    // target (1250) to 1275 over the full 2-minute TeleOp, starting at PLAY.
    // It is added on top of the target, so bumper trims still work.
    private static final double GAME_RAMP_SECS    = 120;   // full 2 min
    private static final double GAME_RAMP_EXTRA   = 25;    // 1250 -> 1275
    private final ElapsedTime gameTimer = new ElapsedTime();
    private double gameRamp = 0;

    // ── Intake ───────────────────────────────────────────────────────────────
    private DcMotorEx leftIntake, rightIntake;
    private static final double INTAKE_POWER = 1.0;

    // ── Linkage ──────────────────────────────────────────────────────────────
    private Servo linkageLeft, linkageRight;
    private static final double LINKAGE_DOWN = 0;
    private static final double LINKAGE_UP   = .37;
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
    // Sequence per R2 press:  wait KICK_DELAY_MS at BASE -> go to KICK ->
    // after KICK_TRAVEL_MS (time for the servo to reach KICK) go back to BASE.
    // A standard servo has no position feedback, so "position reached" is
    // timed. Tune KICK_TRAVEL_MS: raise it if the kicker doesn't fully reach
    // KICK before dropping, lower it if it lingers up there.
    private Servo kicker;
    private static final double KICKER_BASE      = 0.8;
    private static final double KICKER_KICK      = 0.5;
    private static final long   KICK_DELAY_MS    = 500; // ms after R2 press before kicker goes up
    private static final long   KICK_TRAVEL_MS   = 300; // ms for kicker to reach KICK, then it drops

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
    //  Alliance / preset helpers
    // ═════════════════════════════════════════════════════════════════════════

    /** Shooting pose (R2 auto-drive target) for the chosen alliance + preset. */
    private Pose idealPose() {
        return (alliance == Alliance.RED ? RED_SHOOT_POSES : BLUE_SHOOT_POSES)[presetIndex];
    }

    /** Reset pose (X button) for the chosen alliance + preset. */
    private Pose originPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        // Drive is handled by Pedro (motors fl/fr/bl/br are configured in Constants).
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        alliance    = Alliance.RED;
        presetIndex = 0;
        lastPresetBtn = false;
        follower.setPose(originPose());
        alignTarget  = idealPose();
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
        // Flipped: positive power now pulls balls IN (shoot-feed, A) and negative pushes OUT (B).
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linkageLeft  = hardwareMap.get(Servo.class, "linkageleft");
        linkageRight = hardwareMap.get(Servo.class, "linkageright");
        linkageRight.setDirection(Servo.Direction.REVERSE);
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

        telemetry.addLine("Initialized — choose side: gamepad1 dpad UP = RED, DOWN = BLUE");
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  init_loop — pick the alliance with gamepad 1 dpad before PLAY
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init_loop() {
        if (gamepad1.dpad_up)   alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;

        // Keep the starting pose in sync with the chosen side.
        follower.setPose(originPose());

        telemetry.addLine("Gamepad 1:  dpad UP = RED   |   dpad DOWN = BLUE");
        telemetry.addData("Alliance", alliance);
        Pose o = originPose();
        Pose s = idealPose();
        telemetry.addData("Start pose", "x=%.1f  y=%.1f  heading=%.0f deg",
                o.x(), o.y(), Math.toDegrees(o.heading()));
        telemetry.addData("Shoot pose", "x=%.1f  y=%.1f  heading=%.0f deg",
                s.x(), s.y(), Math.toDegrees(s.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Lock in the chosen side's starting pose.
        follower.setPose(originPose());
        alignTarget = idealPose();
        gameTimer.reset();   // game-long shooter ramp starts at PLAY
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        currentArtboardState = 0;

        // ── Preset toggle (gamepad 1 B) ──────────────────────────────────────
        // Switches BOTH the shooting pose and the reset pose for this alliance.
        boolean presetBtn = gamepad1.b;
        if (presetBtn && !lastPresetBtn) presetIndex = 1 - presetIndex;
        lastPresetBtn = presetBtn;

        // ── Drive ────────────────────────────────────────────────────────────
        double stickY  = -gamepad1.left_stick_y;
        double stickX  =  gamepad1.left_stick_x;
        double stickRx =  gamepad1.right_stick_x;

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        // Gamepad 1: X resets pose, R2 starts/cancels auto-align.
        updateAlign();

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

        // Shoot when gamepad 2 R2 is held, OR automatically once the robot has
        // arrived at the shooting pose with gamepad 1 R2 still held (ARRIVED only
        // lasts while gamepad 1 R2 is down). Everything below (hood, kicker delay,
        // blocker, intake feed) keys off this one flag, so auto-shoot behaves
        // exactly like a gamepad 2 R2 press.
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD
                || alignState == AlignState.ARRIVED;

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

        // Boost: 0 -> BOOST_MAX while R2 held, back to 0 on release.
        double boostDt = boostTimer.seconds();
        boostTimer.reset();
        double boostTarget = shooting ? BOOST_MAX : 0;
        if (shooterBoost < boostTarget) {
            shooterBoost = Math.min(boostTarget, shooterBoost + BOOST_RATE * boostDt);
        } else if (shooterBoost > boostTarget) {
            shooterBoost = Math.max(boostTarget, shooterBoost - BOOST_RATE * boostDt);
        }

        // Slow game-long ramp: 0 -> GAME_RAMP_EXTRA over GAME_RAMP_SECS, then holds.
        gameRamp = GAME_RAMP_EXTRA * Math.min(1.0, gameTimer.seconds() / GAME_RAMP_SECS);

        double commandedVelocity = shooterOn ? (shooterVelocity + shooterBoost + gameRamp) : 0;
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

        // Kicker window: goes UP once KICK_DELAY_MS has elapsed from the R2 press,
        // then comes back DOWN after KICK_TRAVEL_MS. It never goes up again until
        // R2 is released and pressed again (that resets kickTimer).
        double kickMs = kickTimer.milliseconds();
        boolean kickReady = shooting
                && kickMs >= KICK_DELAY_MS
                && kickMs <  KICK_DELAY_MS + KICK_TRAVEL_MS;

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
        telemetry.addData("Game ramp", "+%.1f ticks/sec  (%.0f s elapsed)", gameRamp, gameTimer.seconds());
        telemetry.addData("Shooting (R2)", shooting ? "YES" : "no");
        if (shooting) {
            String kickStatus = kickReady ? "(UP)"
                    : (kickMs < KICK_DELAY_MS ? "(waiting)" : "(done — release R2 to re-arm)");
            telemetry.addData("Kick delay", "%.0f / %d ms  %s",
                    Math.min(kickMs, KICK_DELAY_MS), KICK_DELAY_MS, kickStatus);
        }
        telemetry.addData("Intake",  "%s", intakeMode);
        telemetry.addData("Blocker", blockerIsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker",  (shooting && kickReady) ? "KICK" : "BASE");
        telemetry.addData("Ball dist (in)", "%.1f  %s", ballDist, ballPresent ? "BALL" : "empty");
        telemetry.addData("Linkage", linkageUp ? "UP" : "DOWN");
        telemetry.addData("Hood",    "%.2f  (LOWEST=%.2f START=%.2f)", hood.getPosition(), HOOD_LOWEST, HOOD_START);
        telemetry.addLine("---");
        telemetry.addData("Alliance / preset", "%s  #%d", alliance, presetIndex);
        Pose pose = follower.pose();
        telemetry.addData("Align", "%s  busy=%b  mode=%s", alignState, follower.isBusy(), follower.mode());
        telemetry.addData("Pose", "x=%.1f  y=%.1f  heading=%.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to target", "%.1f in", distanceToTarget());
        telemetry.addData("Route", alignState == AlignState.MANUAL ? "-" :
                (routeCorners > 0 ? "curving around structures (" + routeCorners + " guide pts)" : "straight"));

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
    //  Auto-align to the shooting pose (gamepad 1)
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Gamepad 1 X = reset pose to the current reset pose. Gamepad 1 R2 (hold)
     * = auto-drive to the current shooting pose. Only one thing owns the
     * drivetrain at a time:
     *   MANUAL   — driver (plain stick drive, no position hold)
     *   ALIGNING — Ivy is driving the path
     *   ARRIVED  — path done; Pedro holds the target while R2 stays held.
     *              Release R2 to get driver control back.
     */
    private void updateAlign() {
        // X: reset pose once per press, only while the driver has control.
        boolean xHeld = gamepad1.x;
        if (xHeld && !resetWasHeld && driverHasControl()) {
            follower.setPose(originPose());
        }
        resetWasHeld = xHeld;

        boolean alignHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean rising    = alignHeld && !alignWasHeld;
        alignWasHeld = alignHeld;

        switch (alignState) {

            case MANUAL:
                // Only a fresh press starts an alignment.
                if (rising) {
                    alignTarget = idealPose();
                    scheduleRoute(alignTarget);
                    alignStarted = false;
                    alignState = AlignState.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!alignHeld) {
                    // Driver let go of R2: hand control straight back.
                    Scheduler.reset();   // clears ALL scheduled Ivy commands
                    alignState = AlignState.MANUAL;
                } else if (follower.isBusy()) {
                    alignStarted = true;     // path is genuinely running
                } else if (alignStarted) {
                    // Reached the target via R2: now (and only now) hold it.
                    schedule(hold(follower, alignTarget));
                    alignState = AlignState.ARRIVED;
                }
                // !started and !busy: command queued, not started yet. Wait.
                break;

            case ARRIVED:
                // Pedro holds the target for as long as R2 stays held.
                // Releasing R2 hands control back to the driver.
                if (!alignHeld) {
                    Scheduler.reset();   // stops the hold
                    alignState = AlignState.MANUAL;
                }
                break;
        }
    }

    /** True only in MANUAL. While ALIGNING Ivy drives; while ARRIVED Pedro holds the target. */
    private boolean driverHasControl() {
        return alignState == AlignState.MANUAL;
    }

    /** Straight line from the robot's current pose to the given target (heading turns as it drives). */
    private Path pathTo(Pose target) {
        Pose currentPose = follower.pose();
        return line(currentPose, target).linear(currentPose, target);
    }

    /**
     * Schedules the drive to the target. Straight if the way is clear; otherwise
     * one continuous rounded path around (or between) the structures.
     */
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
        schedule(follow(follower, pathTo(target)));   // clear, or no safe route: straight
    }

    /**
     * Builds the dodge path: lines joined by quadratic-Bezier rounded corners,
     * with the heading turning steadily from the current heading to the target's.
     * Returns null if no safe route exists.
     */
    private Path buildCurvedPath(Pose start, Pose target) {
        double pad = ROBOT_MARGIN + CORNER_RADIUS * 0.5;   // rounding cuts in by at most ~R/2
        for (int attempt = 0; attempt < PAD_TRIES; attempt++, pad += PAD_STEP) {
            List<double[]> corners = findCorners(start, target, pad);
            if (corners == null || corners.isEmpty()) return null;

            List<double[]> pts = new ArrayList<>();
            pts.add(new double[]{start.x(), start.y()});
            pts.addAll(corners);
            pts.add(new double[]{target.x(), target.y()});

            List<double[][]> segs = roundedSegments(pts);
            if (!segmentsClear(segs, start, target)) continue;   // cut too close: widen, retry

            // Heading: spread the turn along the path in proportion to distance.
            double h0    = start.heading();
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
                    Pose m = p.of(sg[1][0], sg[1][1], Math.toDegrees(hA));   // corner = control point
                    parts.add(curve(a, m, b).linear(a, b));
                }
            }
            routeCorners = corners.size();
            return path(parts.toArray(new Path[0]));
        }
        return null;
    }

    /**
     * Turns the polyline start -> corners -> target into segments: straight runs
     * (2 points) and rounded corners (3 points: entry, corner as control, exit).
     */
    private List<double[][]> roundedSegments(List<double[]> pts) {
        List<double[][]> out = new ArrayList<>();
        double[] cur = pts.get(0);
        for (int k = 1; k < pts.size() - 1; k++) {
            double[] a = pts.get(k - 1), c = pts.get(k), b = pts.get(k + 1);
            double lenIn  = Math.hypot(c[0] - a[0], c[1] - a[1]);
            double lenOut = Math.hypot(b[0] - c[0], b[1] - c[1]);
            double rIn  = Math.min(CORNER_RADIUS, lenIn  / 2);
            double rOut = Math.min(CORNER_RADIUS, lenOut / 2);
            double[] entry = {c[0] - (c[0] - a[0]) / lenIn * rIn,  c[1] - (c[1] - a[1]) / lenIn * rIn};
            double[] exit  = {c[0] + (b[0] - c[0]) / lenOut * rOut, c[1] + (b[1] - c[1]) / lenOut * rOut};
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

    // ═════════════════════════════════════════════════════════════════════════
    //  Structure dodging — route planner
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Picks which padded zone corners the robot should wrap around, in order.
     * Visibility graph over start, target and every usable corner, shortest path
     * by Dijkstra. Because both rectangles' corners are in the graph, the route
     * goes between the two structures whenever the gap is wide enough.
     * Returns null if no route exists.
     */
    private List<double[]> findCorners(Pose start, Pose target, double pad) {
        // Node 0 = start, node 1 = target, then every usable padded corner.
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{start.x(),  start.y()});
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
        Collections.reverse(chain);          // 0 (start) ... 1 (target)
        List<double[]> out = new ArrayList<>();
        for (int i = 1; i < chain.size() - 1; i++) out.add(nodes.get(chain.get(i)));
        return out;
    }

    /**
     * Samples every segment of the planned path and checks it never comes within
     * ROBOT_MARGIN of a structure. A zone that already holds the start or target
     * is skipped so the robot can always drive out of it.
     */
    private boolean segmentsClear(List<double[][]> segs, Pose start, Pose target) {
        final double eps = 0.01;
        for (double[][] sg : segs) {
            for (int i = 0; i <= CURVE_SAMPLES; i++) {
                double[] pt = bezierPoint(sg, (double) i / CURVE_SAMPLES);
                for (double[] z : KEEP_OUT_ZONES) {
                    if (!active(z)) continue;
                    double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
                    double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
                    if (inside(start.x(),  start.y(),  x0, y0, x1, y1)
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
        final double eps = 0.01;   // so points exactly on a padded edge count as outside
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
            double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
            // If an end is already inside this zone (robot hugging it, or target
            // inside it) don't let that zone block the leg, or we could never leave.
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

    /** A zone still left as {0,0,0,0} is a placeholder and is ignored. */
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

    /** Distance in inches from the robot to the current shooting pose, for telemetry. */
    private double distanceToTarget() {
        Pose target = idealPose();
        return Math.hypot(target.x() - follower.pose().x(),
                target.y() - follower.pose().y());
    }
}