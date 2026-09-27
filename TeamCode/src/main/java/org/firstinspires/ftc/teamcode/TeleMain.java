package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  TeleMain.java  —  Plain hardware TeleOp. No Pedro Pathing, no odometry,
//                    no follower, no NextFTC. Just motors, servos, one sensor.
//
//  HARDWARE (names must match the Robot Configuration on the Driver Hub)
//    4x drive motors    — fl, fr, bl, br    (mecanum)
//    2x shooter motors  — ls, rs            (rs REVERSED so both spin the same way)
//    2x intake motors   — li, ri            (ri REVERSED so both pull inward)
//    2x linkage servos  — linkageleft, linkageright  (always same position)
//    1x blocker servo   — blocker           (toggles OPEN / CLOSED)
//    1x hood servo      — hood              (driven by R2 — see HOOD section below)
//    1x kicker servo    — kicker            (BASE / KICK — pushes last ball up)
//    1x distance sensor — distance          (REV 2m or Color Sensor V3 both work,
//                                             telemetry only — no longer drives the kicker)
//    1x Prism light     — prism             (GoBilda Prism driver, status animations)
//
//  CONTROLS
//    Gamepad 1
//      left stick        — drive / strafe
//      right stick X     — rotate
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
//                          immediately. The kicker's timing depends on the
//                          distance sensor: if no ball is seen the moment
//                          R2 is pressed, the kicker fires right away. If a
//                          ball IS seen, the kicker waits for the sensor to
//                          go clear (ball feeds past it), THEN waits an
//                          additional KICK_DELAY_MS (750ms), THEN fires.
//                          Release R2 and the kicker snaps straight back to
//                          BASE; this whole check runs fresh on every new
//                          R2 press. Holding R2 also ramps the shooter up
//                          by +BOOST_MAX ticks/sec over 1 second (on top of
//                          the HIGH/LOW target); releasing R2 ramps that
//                          boost back down over 1 second too.
//                          R2 ALSO drives the hood directly (no separate
//                          button) — see HOOD below.
//
//  HOOD (driven entirely by R2 — no separate button)
//    While R2 is held (shooting == true): hood snaps to HOOD_LOWEST.
//    The instant R2 is released:          hood snaps back to HOOD_START.
//    This is a plain instant setPosition() every loop based on whether R2 is
//    currently past TRIGGER_THRESHOLD — no ramping, no extra controls.
//    TODO: HOOD_LOWEST below is a placeholder — set it to your real tuned
//    value once you've found it on the bench.
//
//  SHOOT SEQUENCE (while R2 held)
//    1. Blocker forced OPEN, intake forced IN (overrides A/B and the Y toggle)
//       — happens the instant R2 crosses the trigger threshold.
//    2. At that same instant, we check the distance sensor once:
//         - No ball seen  -> kicker goes to KICK immediately.
//         - Ball seen     -> kicker stays at BASE and waits for the sensor
//           to report clear (the ball has fed past it). Once clear, it
//           waits KICK_DELAY_MS more, then moves to KICK and stays there
//           for the rest of the hold.
//    3. On release of R2, kicker snaps back to BASE, the blocker returns to
//       whatever the Y toggle had it at, the hood snaps back to HOOD_START,
//       and the next R2 press re-runs this whole check fresh.
//    NOTE: R2 does not turn the shooter on by itself — spin it up with dpad up
//    first so the flywheels are at speed when balls arrive.
//
//  SHOOTER VELOCITY CONTROL
//    The shooter runs closed-loop on encoder velocity (ticks/sec) using the
//    PIDF gains you found with FlywheelTuner, instead of raw setPower().
//    HIGH and LOW velocity are no longer fixed constants — left bumper /
//    right bumper nudge whichever mode (HIGH or LOW) is currently selected
//    up or down by VELOCITY_STEP (100 ticks/sec) so you can fine-tune live
//    on the field. The starting values below are just the initial targets.
//    While R2 (the shoot trigger) is held, the shooter also gets a boost on
//    top of the HIGH/LOW target: it ramps up to +BOOST_MAX ticks/sec over
//    1 second, and ramps back down to the normal target over 1 second after
//    R2 is released (see shooterBoost).
//
//  VOLTAGE / CURRENT MONITORING
//    IMPORTANT — what the FTC SDK actually exposes:
//      - VOLTAGE is only readable per HUB (each Control Hub / Expansion Hub
//        is one VoltageSensor). There is NO way to read voltage at an
//        individual motor, servo, sensor, or the Prism light — the SDK
//        just doesn't expose that. hardwareMap.voltageSensor iterates every
//        hub on the robot; we log each one and the worst (minimum) of them,
//        since that's the one that actually limits you.
//      - CURRENT (amps) IS readable per motor, but only for motors declared
//        as DcMotorEx (which is why fl/fr/bl/br/li/ri were switched to
//        DcMotorEx below, alongside the shooter motors which already were).
//        Servos and I2C sensors (distance sensor, Prism) report neither
//        voltage nor current in the SDK — there's nothing to log for them.
//    Rough numbers to judge the telemetry against (typical FTC 12V battery):
//      - Fresh, fully-charged battery: ~12.5-13.0V at rest.
//      - Normal mid-match voltage under load: ~11.0-12.5V.
//      - Below ~9.5-10V: hubs risk brownout / erratic behavior — swap the battery.
//      - Drive motor current: ~0.5-1A per motor cruising, ~3-4A pushing hard
//        against something; a whole drivetrain spiking toward ~15-20A (which
//        is where packs are typically fused) is the danger zone.
//      - Shooter/intake motors will show higher current while spinning up
//        from a stop or under heavy load — that alone isn't a problem.
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

@TeleOp(name = "TeleMain", group = "TeleOp")
public class TeleMain extends OpMode {

    // ── Drive ────────────────────────────────────────────────────────────────
    // DcMotorEx (not plain DcMotor) so we can read current draw for telemetry.
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;

    // Final tuned numbers from FlywheelTuner (ShooterRpmTuner run).
    private static final double SHOOTER_P      = 1.5;    // tuned P gain
    private static final double SHOOTER_F_LEFT  = 12.77; // tuned F, left shooter motor
    private static final double SHOOTER_F_RIGHT = 12.77; // tuned F, right shooter motor
    private GoBildaPrismDriver prism;


    // Mutable now — left/right bumper adjusts whichever mode (HIGH or LOW)
    // is currently selected by VELOCITY_STEP each press.
    private double shooterHighVelocity = 1500; // ticks/sec, same units as FlywheelTuner's highVelocity
    private double shooterLowVelocity  = 1500; // ticks/sec, same units as FlywheelTuner's lowVelocity
    private static final double VELOCITY_STEP = 100; // ticks/sec per bumper press
    private static final double VELOCITY_MIN  = 0;   // floor so you can't drive it negative

    private boolean shooterOn   = false;
    private boolean shooterHigh = false;

    // Trigger boost: while R2 (the shoot trigger) is held, the shooter
    // target climbs by up to BOOST_MAX ticks/sec, ramping in over 1 second
    // (BOOST_RATE per second). Release R2 and it ramps back down to the
    // normal HIGH/LOW target over 1 second the same way.
    private double shooterBoost = 0;
    private static final double BOOST_MAX  = 550; // ticks/sec added at full boost
    private static final double BOOST_RATE = 550; // ticks/sec change per second (0->550 in 1s)
    private final ElapsedTime boostTimer = new ElapsedTime();

    // ── Intake ───────────────────────────────────────────────────────────────
    // DcMotorEx (not plain DcMotor) so we can read current draw for telemetry.
    private DcMotorEx leftIntake, rightIntake;
    private static final double INTAKE_POWER = 1.0;

    // ── Linkage (2 mirrored servos, always same position) ────────────────────
    private Servo linkageLeft, linkageRight;
    private static final double LINKAGE_DOWN = 0.0;
    private static final double LINKAGE_UP   = 1.0;
    private boolean linkageUp = false;
    private int currentArtboardState = -1;

    // ── Blocker ──────────────────────────────────────────────────────────────
    private Servo blocker;
    private static final double BLOCKER_CLOSED = 1.0;
    private static final double BLOCKER_OPEN   = 0.6;
    private boolean blockerOpen = false;   // Y-toggle state (R2 overrides while held)

    // ── Hood — driven directly by R2, no separate button ──────────────────────
    private Servo hood;
    private static final double HOOD_START   = 0.5;  // resting position (R2 not held)
    private static final double HOOD_LOWEST  = 0.35; // TODO — set to your real tuned value; position while R2 is held

    // ── Kicker (2 positions: BASE and KICK) — delayed after R2 press ─────────
    private Servo kicker;

    // TUNE these two on the robot: BASE = resting under the ball path,
    // KICK = fully pushed up into the shooter.
    private static final double KICKER_BASE = 0.8;
    private static final double KICKER_KICK = 0.5;

    // How long to wait, once the ball clears the sensor, before the kicker
    // fires (see the kick state machine below).
    private static final long KICK_DELAY_MS = 750;

    // Kick state machine, reset fresh on every new R2 press:
    //   IMMEDIATE   — no ball was seen when R2 was pressed -> kick right away.
    //   WAIT_CLEAR  — a ball WAS seen -> wait for the sensor to see nothing
    //                 (ball has fed past it), then...
    //   DELAYING    — ...start the KICK_DELAY_MS countdown, then kick.
    private static final int KICK_STATE_IMMEDIATE  = 0;
    private static final int KICK_STATE_WAIT_CLEAR = 1;
    private static final int KICK_STATE_DELAYING   = 2;
    private int kickState = KICK_STATE_IMMEDIATE;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean lastShooting = false;

    // ── Distance sensor (telemetry only) ─────────────────────────────────────
    private DistanceSensor ballSensor;
    private static final double BALL_PRESENT_INCHES = 2.0;

    // ── Voltage monitoring ───────────────────────────────────────────────────
    // Rough thresholds for the "status" label next to the min hub voltage.
    private static final double VOLTAGE_GOOD_MIN     = 12.0; // >= this: fully healthy
    private static final double VOLTAGE_OK_MIN       = 11.0; // >= this: normal sag under load
    private static final double VOLTAGE_LOW_MIN      = 9.5;  // >= this: swap the battery soon
    // below VOLTAGE_LOW_MIN: CRITICAL — hub brownout risk

    // ── Shoot trigger ────────────────────────────────────────────────────────
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    private boolean lastDpadUp    = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown  = false;
    private boolean lastX         = false;
    private boolean lastY         = false;
    private boolean lastLeftBumper  = false;
    private boolean lastRightBumper = false;

    // ═════════════════════════════════════════════════════════════════════════
    //  init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        // ── Drive motors ─────────────────────────────────────────────────────
        frontLeft  = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft   = hardwareMap.get(DcMotorEx.class, "bl");
        backRight  = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        currentArtboardState = 0; // Track that we're showing artboard 0

        // ── Shooter motors — closed-loop velocity control ───────────────────
        leftShooter  = hardwareMap.get(DcMotorEx.class, "ls");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rs");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients leftShooterPIDF  = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT);
        PIDFCoefficients rightShooterPIDF = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_RIGHT);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftShooterPIDF);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightShooterPIDF);

        // ── Intake motors ────────────────────────────────────────────────────
        leftIntake  = hardwareMap.get(DcMotorEx.class, "li");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ── Linkage servos ───────────────────────────────────────────────────
        linkageLeft  = hardwareMap.get(Servo.class, "linkageleft");
        linkageRight = hardwareMap.get(Servo.class, "linkageright");
        // linkageRight.setDirection(Servo.Direction.REVERSE); // if mirrored
        linkageUp = false;
        setLinkagePosition(LINKAGE_DOWN);

        // ── Blocker servo ────────────────────────────────────────────────────
        blocker = hardwareMap.get(Servo.class, "blocker");
        blockerOpen = false;
        blocker.setPosition(BLOCKER_CLOSED);

        // ── Hood servo ───────────────────────────────────────────────────────
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(HOOD_START);

        // ── Kicker servo ─────────────────────────────────────────────────────
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_BASE);

        // ── Distance sensor ──────────────────────────────────────────────────
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
        // ── Drive (gamepad 1) ────────────────────────────────────────────────
        double stickY  = -gamepad1.left_stick_y;
        double stickX  =  gamepad1.left_stick_x;
        double stickRx =  gamepad1.right_stick_x;

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        double fl = stickY + stickX + stickRx;
        double fr = stickY - stickX - stickRx;
        double bl = stickY - stickX + stickRx;
        double br = stickY + stickX - stickRx;

        double maxPow = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br))));
        if (maxPow > 1.0) { fl /= maxPow; fr /= maxPow; bl /= maxPow; br /= maxPow; }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // ── Shooter mode (gamepad 2 dpad) ────────────────────────────────────
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

        // Read the shoot trigger early — the flywheel boost AND the hood
        // below both need it before the SHOOT section later in the loop does.
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD;

        // ── Hood — driven directly by R2, no separate button ──────────────────
        // Instant snap every loop: HOOD_LOWEST while R2 is held, HOOD_START
        // the instant it's released. (dpad_down above still works too — it's
        // redundant while not shooting since this line sets the same value
        // anyway, and it's overridden by this line the instant you shoot.)
        hood.setPosition(shooting ? HOOD_LOWEST : HOOD_START);

        // ── Flywheel speed trim (gamepad 2 bumpers) ──────────────────────────
        // Nudges whichever mode (HIGH or LOW) is currently selected by
        // VELOCITY_STEP ticks/sec per press. Works whether the shooter is
        // spinning or not, so you can dial it in before or during a match.
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

        // Trigger boost: ramp shooterBoost toward BOOST_MAX while R2 is
        // held, and back toward 0 once it's released — both over 1 second
        // at BOOST_RATE ticks/sec per second.
        double boostDt = boostTimer.seconds();
        boostTimer.reset();
        double boostTarget = shooting ? BOOST_MAX : 0;
        if (shooterBoost < boostTarget) {
            shooterBoost = Math.min(boostTarget, shooterBoost + BOOST_RATE * boostDt);
        } else if (shooterBoost > boostTarget) {
            shooterBoost = Math.max(boostTarget, shooterBoost - BOOST_RATE * boostDt);
        }

        // Boost only means anything while the shooter is actually on.
        double commandedVelocity = shooterOn ? (shooterVelocity + shooterBoost) : 0;

        leftShooter.setVelocity(commandedVelocity);
        rightShooter.setVelocity(commandedVelocity);

        // ── Blocker toggle (gamepad 2 Y) — stores state; applied below ───────
        boolean curY = gamepad2.y;
        if (curY && !lastY) blockerOpen = !blockerOpen;
        lastY = curY;

        // ── Ball sensor (telemetry only) ──────────────────────────────────────
        double ballDist = ballSensor.getDistance(DistanceUnit.INCH);
        boolean ballPresent = !Double.isNaN(ballDist) && ballDist <= BALL_PRESENT_INCHES;

        // ── SHOOT (gamepad 2 R2, held) ───────────────────────────────────────
        // ("shooting" was already read above, before the flywheel boost and
        // hood logic.)

        // Rising edge of R2: decide the starting kick state from whether a
        // ball is sitting in front of the sensor RIGHT NOW.
        //   - No ball seen  -> IMMEDIATE (kick right away, nothing to wait for)
        //   - Ball seen     -> WAIT_CLEAR (wait for the ball to feed past the
        //                      sensor first, then delay KICK_DELAY_MS, then kick)
        if (shooting && !lastShooting) {
            kickState = ballPresent ? KICK_STATE_WAIT_CLEAR : KICK_STATE_IMMEDIATE;
        }
        lastShooting = shooting;

        // Once we're waiting for the ball to clear, the moment the sensor
        // reports empty we start the KICK_DELAY_MS countdown.
        if (kickState == KICK_STATE_WAIT_CLEAR && !ballPresent) {
            kickState = KICK_STATE_DELAYING;
            kickTimer.reset();
        }

        boolean kickReady = shooting && (
                kickState == KICK_STATE_IMMEDIATE
                        || (kickState == KICK_STATE_DELAYING && kickTimer.milliseconds() >= KICK_DELAY_MS)
        );

        double intakePower;
        String intakeMode;

        if (shooting) {
            // Intake feeds balls up and blocker opens the instant R2 is
            // pressed. The kicker itself waits KICK_DELAY_MS before moving
            // to KICK, then holds there for the rest of the hold.
            intakePower = INTAKE_POWER;
            intakeMode  = "SHOOT-FEED";
            blocker.setPosition(BLOCKER_OPEN);

            kicker.setPosition(kickReady ? KICKER_KICK : KICKER_BASE);
        } else {
            // Not shooting — normal A/B intake, blocker follows Y toggle,
            // kicker parked at base.
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

        // ── Linkage servos (gamepad 2 X, toggle) ─────────────────────────────
        boolean curX = gamepad2.x;
        if (curX && !lastX) {
            linkageUp = !linkageUp;
            setLinkagePosition(linkageUp ? LINKAGE_UP : LINKAGE_DOWN);
        }
        lastX = curX;

        // ── Telemetry ────────────────────────────────────────────────────────
        boolean blockerIsOpen = shooting || blockerOpen;
        telemetry.addData("Shooter", "%s target=%.0f boost=%.0f commanded=%.0f actual(L/R)=%.0f/%.0f",
                shooterMode, shooterVelocity, shooterBoost, commandedVelocity, leftShooter.getVelocity(), rightShooter.getVelocity());
        telemetry.addData("Shooter HIGH/LOW targets", "%.0f / %.0f", shooterHighVelocity, shooterLowVelocity);
        telemetry.addData("Shooting (R2)", shooting ? "YES" : "no");
        if (shooting) {
            String kickStateName = (kickState == KICK_STATE_IMMEDIATE) ? "IMMEDIATE (no ball at press)"
                    : (kickState == KICK_STATE_WAIT_CLEAR) ? "WAITING FOR CLEAR"
                    : "DELAYING";
            telemetry.addData("Kick state", kickStateName);
            if (kickState == KICK_STATE_DELAYING) {
                telemetry.addData("Kick delay", "%.0f / %d ms  %s",
                        Math.min(kickTimer.milliseconds(), KICK_DELAY_MS), KICK_DELAY_MS,
                        kickReady ? "(FIRING)" : "(waiting)");
            }
        }
        telemetry.addData("Intake",   "%s", intakeMode);
        telemetry.addData("Blocker",  blockerIsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker",   kickReady ? "KICK" : "BASE");
        telemetry.addData("Ball dist (in)", "%.1f  %s", ballDist, ballPresent ? "BALL" : "empty");
        telemetry.addData("Linkage",  linkageUp ? "UP" : "DOWN");
        telemetry.addData("Hood",     "%.2f  (LOWEST=%.2f START=%.2f)", hood.getPosition(), HOOD_LOWEST, HOOD_START);
        telemetry.addLine("---");
        telemetry.addData("Drive fl/fr", "%.2f / %.2f", fl, fr);
        telemetry.addData("Drive bl/br", "%.2f / %.2f", bl, br);

        // ── Voltage / current ────────────────────────────────────────────────
        // NOTE: the SDK only exposes voltage per-HUB, and current only for
        // DcMotorEx motors. Servos, the distance sensor, and the Prism light
        // have no voltage/current reading available — see header comment.
        telemetry.addLine("--- Voltage / Current ---");
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) minVoltage = Math.min(minVoltage, v);
            String hubName = String.join("/", hardwareMap.getNamesOf(vs));
            telemetry.addData("Hub voltage [" + hubName + "]", "%.2f V", v);
        }
        String voltageStatus;
        if (minVoltage >= VOLTAGE_GOOD_MIN)      voltageStatus = "GOOD (should be ~12.0-13.0V fresh)";
        else if (minVoltage >= VOLTAGE_OK_MIN)   voltageStatus = "OK (normal sag under load)";
        else if (minVoltage >= VOLTAGE_LOW_MIN)  voltageStatus = "LOW - swap battery soon";
        else                                       voltageStatus = "CRITICAL - brownout risk!";
        telemetry.addData("Min battery voltage", "%.2f V  [%s]", minVoltage, voltageStatus);

        telemetry.addData("Drive current fl/fr/bl/br (A)", "%.2f / %.2f / %.2f / %.2f  (expect ~0.5-1A cruising, ~3-4A hard push)",
                frontLeft.getCurrent(CurrentUnit.AMPS), frontRight.getCurrent(CurrentUnit.AMPS),
                backLeft.getCurrent(CurrentUnit.AMPS), backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake current li/ri (A)", "%.2f / %.2f",
                leftIntake.getCurrent(CurrentUnit.AMPS), rightIntake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Shooter current ls/rs (A)", "%.2f / %.2f  (spikes while spinning up are normal)",
                leftShooter.getCurrent(CurrentUnit.AMPS), rightShooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine("Servos, distance sensor & Prism light: no voltage/current reading exists in the SDK");

        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  stop  —  park everything safely
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        setLinkagePosition(LINKAGE_DOWN);
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);
        hood.setPosition(HOOD_START);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Helper — drives BOTH linkage servos to the same position, always.
    // ═════════════════════════════════════════════════════════════════════════
    private void setLinkagePosition(double position) {
        linkageLeft.setPosition(position);
        linkageRight.setPosition(position);
    }
}