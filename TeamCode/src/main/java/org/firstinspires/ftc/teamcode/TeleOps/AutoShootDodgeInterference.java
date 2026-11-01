package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
//  AutoShootDodgeInterference.java
//
//  DESCRIPTION:
//    Extension of AutoShootDodge.java.
//    1. Normal behavior: Drive manually, hold Gamepad 1 R2 to auto-align to shooting pose.
//    2. INTERFERENCE DETECTION & GOAL TRACKING:
//       If an enemy robot or physical obstacle interferes/blocks the robot while
//       aligning (detected via stall timer, distance check, or Gamepad 1 L2 trigger),
//       the robot stops pushing against the obstacle, calculates the exact angle to the
//       Goal Target from its CURRENT field position, rotates in place to face the goal,
//       and fires into the goal!
//
//  CONTROLS (Gamepad 1)
//    left stick        — drive / strafe
//    right stick X     — rotate
//    B (press)         — switch preset (0 <-> 1)
//    R2 (hold)         — auto-drive to shooting pose. If blocked by interference,
//                        automatically rotates to track goal and shoots!
//    L2 (hold)         — manual trigger to force Interference Goal Tracking immediately.
//    X                 — reset pose to origin.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name = "AutoShootDodge + Interference Track", group = "TeleOp")
public class AutoShootDodgeInterference extends OpMode {

    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Align States including Interference Detection ────────────────────────
    private enum AlignState {
        MANUAL,
        ALIGNING,
        ARRIVED,
        INTERFERENCE_AIMING,
        INTERFERENCE_SHOOTING
    }

    private static final PoseFactory p = PoseFactory.degrees();

    // ── Alliance + Preset Poses ──────────────────────────────────────────────
    private enum Alliance { RED, BLUE }

    public static final Pose[] RED_SHOOT_POSES = {
            p.of(56, 14, -90),
            p.of(56, 130, 90)
    };
    public static final Pose[] RED_RESET_POSES = {
            p.of(7.25, 8, -90),
            p.of(7.25, 136, 90)
    };

    public static final Pose[] BLUE_SHOOT_POSES = {
            p.of(83, 14, -90),
            p.of(83, 130, 90)
    };
    public static final Pose[] BLUE_RESET_POSES = {
            p.of(135.25, 8, -90),
            p.of(135.25, 136, 90)
    };

    // ── High Goal Coordinates ────────────────────────────────────────────────
    public static final double RED_GOAL_X  = 62.6852;
    public static final double RED_GOAL_Y  = 86.5374;
    public static final double BLUE_GOAL_X = 144.0 - 62.6852;
    public static final double BLUE_GOAL_Y = 86.5374;

    private Alliance alliance    = Alliance.RED;
    private int      presetIndex = 0;
    private boolean  lastPresetBtn = false;

    private AlignState alignState    = AlignState.MANUAL;
    private boolean    alignWasHeld  = false;
    private boolean    resetWasHeld  = false;
    private boolean    alignStarted  = false;
    private Pose       alignTarget   = RED_SHOOT_POSES[0];

    // ── Interference Detection Settings ──────────────────────────────────────
    private final ElapsedTime alignTimer = new ElapsedTime();
    private static final double INTERFERENCE_TIMEOUT_SECS = 2.0; // If blocked for 2s while driving to target
    private static final double STUCK_DISTANCE_THRESHOLD  = 5.0; // Robot is still > 5 inches from goal pose
    private static final double AIM_TOLERANCE_DEG         = 3.0; // Heading tolerance when aiming at goal

    // ── Structure Dodging ────────────────────────────────────────────────────
    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91},
            {91, 50, 96, 91}
    };

    private static final double ROBOT_MARGIN = 9.0;
    private static final double CORNER_RADIUS = 10.0;
    private static final int    CURVE_SAMPLES = 20;
    private static final double PAD_STEP      = 1.5;
    private static final int    PAD_TRIES     = 10;
    private static final double FIELD_MIN     = 0;
    private static final double FIELD_MAX     = 144;

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

    private double shooterBoost = 0;
    private static final double BOOST_MAX       = 0;
    private static final double BOOST_RAMP_SECS = 0;
    private static final double BOOST_RATE      = BOOST_MAX / BOOST_RAMP_SECS;
    private final ElapsedTime boostTimer = new ElapsedTime();

    private static final double GAME_RAMP_SECS  = 120;
    private static final double GAME_RAMP_EXTRA = 25;
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

    // ── Blocker ──────────────────────────────────────────────────────────────
    private Servo blocker;
    private static final double BLOCKER_CLOSED = 1.0;
    private static final double BLOCKER_OPEN   = 0.6;
    private boolean blockerOpen = false;

    // ── Hood ─────────────────────────────────────────────────────────────────
    private Servo hood;
    private static final double HOOD_START  = 0.5;
    private static final double HOOD_LOWEST = 0.35;

    // ── Kicker ───────────────────────────────────────────────────────────────
    private Servo kicker;
    private static final double KICKER_BASE    = 0.8;
    private static final double KICKER_KICK    = 0.5;
    private static final long   KICK_DELAY_MS  = 500;
    private static final long   KICK_TRAVEL_MS = 300;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean lastShooting = false;

    // ── Distance Sensor ──────────────────────────────────────────────────────
    private DistanceSensor ballSensor;
    private static final double BALL_PRESENT_INCHES = 2.0;

    private static final double TRIGGER_THRESHOLD = 0.5;

    private boolean lastDpadUp      = false;
    private boolean lastDpadLeft    = false;
    private boolean lastDpadRight   = false;
    private boolean lastDpadDown    = false;
    private boolean lastX           = false;
    private boolean lastY           = false;
    private boolean lastLeftBumper  = false;
    private boolean lastRightBumper = false;

    private Pose idealPose() {
        return (alliance == Alliance.RED ? RED_SHOOT_POSES : BLUE_SHOOT_POSES)[presetIndex];
    }

    private Pose originPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    private double getGoalX() {
        return alliance == Alliance.RED ? RED_GOAL_X : BLUE_GOAL_X;
    }

    private double getGoalY() {
        return alliance == Alliance.RED ? RED_GOAL_Y : BLUE_GOAL_Y;
    }

    @Override
    public void init() {
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

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up)   alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;

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
        follower.setPose(originPose());
        alignTarget = idealPose();
        gameTimer.reset();
    }

    @Override
    public void loop() {
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        boolean presetBtn = gamepad1.b;
        if (presetBtn && !lastPresetBtn) presetIndex = 1 - presetIndex;
        lastPresetBtn = presetBtn;

        double stickY  = -gamepad1.left_stick_y;
        double stickX  =  gamepad1.left_stick_x;
        double stickRx =  gamepad1.right_stick_x;

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        // Auto-align & Interference Goal Tracking handler
        updateAlign();

        if (driverHasControl()) {
            double lateral = -stickX;
            double turn    = -stickRx;
            follower.manual(stickY, lateral, turn);
        }

        follower.update();
        Scheduler.execute();

        // ── Shooter Mode ─────────────────────────────────────────────────────
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

        // Trigger shooting if gamepad 2 R2 is held, OR if arrived / tracking goal on interference
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD
                || alignState == AlignState.ARRIVED
                || alignState == AlignState.INTERFERENCE_SHOOTING;

        // ── Hood ─────────────────────────────────────────────────────────────
        hood.setPosition(shooting ? HOOD_LOWEST : HOOD_START);

        // ── Flywheel Speed Trim ───────────────────────────────────────────────
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
        if (!shooterOn && !shooting) {
            shooterVelocity = 0;
            shooterMode     = "OFF";
        } else if (shooterHigh) {
            shooterVelocity = shooterHighVelocity;
            shooterMode     = "HIGH";
        } else {
            shooterVelocity = shooterLowVelocity;
            shooterMode     = "LOW";
        }

        double boostDt = boostTimer.seconds();
        boostTimer.reset();
        double boostTarget = shooting ? BOOST_MAX : 0;
        if (shooterBoost < boostTarget) {
            shooterBoost = Math.min(boostTarget, shooterBoost + BOOST_RATE * boostDt);
        } else if (shooterBoost > boostTarget) {
            shooterBoost = Math.max(boostTarget, shooterBoost - BOOST_RATE * boostDt);
        }

        gameRamp = GAME_RAMP_EXTRA * Math.min(1.0, gameTimer.seconds() / GAME_RAMP_SECS);

        double commandedVelocity = (shooterOn || shooting) ? (shooterVelocity + shooterBoost + gameRamp) : 0;
        leftShooter.setVelocity(commandedVelocity);
        rightShooter.setVelocity(commandedVelocity);

        // ── Blocker Toggle ───────────────────────────────────────────────────
        boolean curY = gamepad2.y;
        if (curY && !lastY) blockerOpen = !blockerOpen;
        lastY = curY;

        double ballDist     = ballSensor.getDistance(DistanceUnit.INCH);
        boolean ballPresent = !Double.isNaN(ballDist) && ballDist <= BALL_PRESENT_INCHES;

        if (shooting && !lastShooting) {
            kickTimer.reset();
        }
        lastShooting = shooting;

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

        // ── Linkage ──────────────────────────────────────────────────────────
        boolean curX = gamepad2.x;
        if (curX && !lastX) {
            linkageUp = !linkageUp;
            setLinkagePosition(linkageUp ? LINKAGE_UP : LINKAGE_DOWN);
        }
        lastX = curX;

        // ── Telemetry ────────────────────────────────────────────────────────
        boolean blockerIsOpen = shooting || blockerOpen;
        telemetry.addData("Shooter", "%s target=%.0f commanded=%.0f actual=%.0f/%.0f",
                shooterMode, shooterVelocity, commandedVelocity,
                leftShooter.getVelocity(), rightShooter.getVelocity());
        telemetry.addData("Shooting (R2)", shooting ? "YES" : "no");
        telemetry.addData("Align State", alignState);
        if (alignState == AlignState.INTERFERENCE_AIMING || alignState == AlignState.INTERFERENCE_SHOOTING) {
            telemetry.addLine("⚠️ INTERFERENCE DETECTED! Tracking goal from current position!");
            telemetry.addData("Goal Target Angle", "%.1f deg", getAngleToGoalDeg());
        }
        telemetry.addData("Intake",  "%s", intakeMode);
        telemetry.addData("Blocker", blockerIsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker",  (shooting && kickReady) ? "KICK" : "BASE");
        telemetry.addData("Ball dist (in)", "%.1f  %s", ballDist, ballPresent ? "BALL" : "empty");
        telemetry.addData("Alliance / Preset", "%s  #%d", alliance, presetIndex);
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x=%.1f  y=%.1f  heading=%.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to target", "%.1f in", distanceToTarget());
        telemetry.update();
    }

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

    /** Calculates angle in degrees from current robot position to the alliance High Goal. */
    private double getAngleToGoalDeg() {
        Pose currentPose = follower.pose();
        double dx = getGoalX() - currentPose.x();
        double dy = getGoalY() - currentPose.y();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /**
     * Auto-align + Interference Goal Tracking state machine.
     */
    private void updateAlign() {
        boolean xHeld = gamepad1.x;
        if (xHeld && !resetWasHeld && driverHasControl()) {
            follower.setPose(originPose());
        }
        resetWasHeld = xHeld;

        boolean alignHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean forceInterferenceHeld = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean rising    = alignHeld && !alignWasHeld;
        alignWasHeld = alignHeld;

        switch (alignState) {

            case MANUAL:
                if (rising) {
                    alignTarget = idealPose();
                    scheduleRoute(alignTarget);
                    alignStarted = false;
                    alignTimer.reset();
                    alignState = AlignState.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!alignHeld) {
                    Scheduler.reset();
                    alignState = AlignState.MANUAL;
                } else {
                    double distRemaining = distanceToTarget();

                    // Check for Interference (enemy bot blocking path, timeout, or manual L2 force)
                    boolean timeStuck = alignTimer.seconds() > INTERFERENCE_TIMEOUT_SECS && distRemaining > STUCK_DISTANCE_THRESHOLD;
                    if (timeStuck || forceInterferenceHeld) {
                        Scheduler.reset();
                        double goalAngleDeg = getAngleToGoalDeg();
                        Pose currentPose = follower.pose();
                        Pose aimGoalPose = p.of(currentPose.x(), currentPose.y(), goalAngleDeg);

                        schedule(hold(follower, aimGoalPose));
                        alignState = AlignState.INTERFERENCE_AIMING;
                    } else if (follower.isBusy()) {
                        alignStarted = true;
                    } else if (alignStarted) {
                        schedule(hold(follower, alignTarget));
                        alignState = AlignState.ARRIVED;
                    }
                }
                break;

            case INTERFERENCE_AIMING:
                if (!alignHeld) {
                    Scheduler.reset();
                    alignState = AlignState.MANUAL;
                } else {
                    double goalAngleDeg = getAngleToGoalDeg();
                    double currentHeadingDeg = Math.toDegrees(follower.pose().heading());
                    double angleDiff = Math.abs(Math.atan2(Math.sin(Math.toRadians(goalAngleDeg - currentHeadingDeg)),
                            Math.cos(Math.toRadians(goalAngleDeg - currentHeadingDeg))));

                    if (Math.toDegrees(angleDiff) < AIM_TOLERANCE_DEG || alignTimer.seconds() > INTERFERENCE_TIMEOUT_SECS + 1.0) {
                        alignState = AlignState.INTERFERENCE_SHOOTING;
                    }
                }
                break;

            case INTERFERENCE_SHOOTING:
            case ARRIVED:
                if (!alignHeld) {
                    Scheduler.reset();
                    alignState = AlignState.MANUAL;
                }
                break;
        }
    }

    private boolean driverHasControl() {
        return alignState == AlignState.MANUAL;
    }

    private Path pathTo(Pose target) {
        Pose currentPose = follower.pose();
        return line(currentPose, target).linear(currentPose, target);
    }

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
        schedule(follow(follower, pathTo(target)));
    }

    private Path buildCurvedPath(Pose start, Pose target) {
        double pad = ROBOT_MARGIN + CORNER_RADIUS * 0.5;
        for (int attempt = 0; attempt < PAD_TRIES; attempt++, pad += PAD_STEP) {
            List<double[]> corners = findCorners(start, target, pad);
            if (corners == null || corners.isEmpty()) return null;

            List<double[]> pts = new ArrayList<>();
            pts.add(new double[]{start.x(), start.y()});
            pts.addAll(corners);
            pts.add(new double[]{target.x(), target.y()});

            List<double[][]> segs = roundedSegments(pts);
            if (!segmentsClear(segs, start, target)) continue;

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
                    Pose m = p.of(sg[1][0], sg[1][1], Math.toDegrees(hA));
                    parts.add(curve(a, m, b).linear(a, b));
                }
            }
            routeCorners = corners.size();
            return path(parts.toArray(new Path[0]));
        }
        return null;
    }

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

    private List<double[]> findCorners(Pose start, Pose target, double pad) {
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
        Collections.reverse(chain);
        List<double[]> out = new ArrayList<>();
        for (int i = 1; i < chain.size() - 1; i++) out.add(nodes.get(chain.get(i)));
        return out;
    }

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

    private boolean isBlocked(double ax, double ay, double bx, double by) {
        final double eps = 0.01;
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
            double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
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

    private static boolean active(double[] z) {
        return !(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 0);
    }

    private static boolean inside(double x, double y, double x0, double y0, double x1, double y1) {
        return x > x0 && x < x1 && y > y0 && y < y1;
    }

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

    private double distanceToTarget() {
        Pose target = idealPose();
        return Math.hypot(target.x() - follower.pose().x(),
                target.y() - follower.pose().y());
    }
}