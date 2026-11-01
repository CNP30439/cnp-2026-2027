package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// ═══════════════════════════════════════════════════════════════════════════════
//  TurtleTracerAuto — Autonomous OpMode (FTC BioBuzz Season)
//
//  SEQUENCE:
//    1. Start Pose: (59.601, 10.167, 90°)
//    2. Drive Shootingpoint1 -> First Shooting Spot (60.0, 25.0, -90°) & Auto-Aim / SHOOT.
//    3. Drive IntakePoint1 -> Intaking location (11.686, 12.037, 0°) with INTAKE ON.
//    4. Drive ShootingPoint2 -> Second Shooting Spot (39.968, 114.176, 0°).
//    5. Auto-Aim at High Goal Target (62.685, 86.537) & SHOOT.
//    6. Drive ParkingPoint -> Final Park Spot (10.050, 114.176, 0°).
//    7. Stop all subsystems.
// ═══════════════════════════════════════════════════════════════════════════════
@Autonomous(name = "TurtleTracerAuto", group = "Autonomous")
public class TurtleTracerAuto extends LinearOpMode {

    private Follower follower;

    // ── High Goal Coordinates ────────────────────────────────────────────────
    public static final double GOAL_X = 62.6852;
    public static final double GOAL_Y = 86.5374;

    // ── Hardware Declarations (From TeleOps) ─────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx leftIntake, rightIntake;
    private Servo blocker, kicker, hood;

    // ── Shooter Settings (PIDF & Velocities) ─────────────────────────────────
    private static final double SHOOTER_P       = 1.5;
    private static final double SHOOTER_F_LEFT  = 18.3237;
    private static final double SHOOTER_F_RIGHT = 18.3237;
    private static final double SHOOT_VEL       = 1250;

    // ── Servo & Intake Parameters ────────────────────────────────────────────
    private static final boolean INTAKE_FLIPPED = true;
    private static final double INTAKE_POWER    = 1.0;

    private static final double BLOCKER_OPEN   = 0.6;
    private static final double BLOCKER_CLOSED = 1.0;

    private static final double KICKER_BASE = 0.8;
    private static final double KICKER_KICK = 0.5;

    private static final double HOOD_START  = 0.5;
    private static final double HOOD_LOWEST = 0.35;

    private static final long SPINUP_SETTLE_MS = 500;  // 0.5s flywheel settle time
    private static final long KICK_DELAY_MS    = 500;  // Intake feed delay before kicker fires
    private static final long SHOOT_HOLD_MS    = 1500; // Kicker hold duration to clear rings

    private Paths paths;

    // ── Path Definitions ─────────────────────────────────────────────────────
    public static class Paths {
        private static final PoseFactory p = PoseFactory.degrees();
        public Path Shootingpoint1;
        public Path IntakePoint1;
        public Path ShootingPoint2;
        public Path ParkingPoint;

        public Paths() {
            // Shooting Point #1: Align to (60.0, 25.0, -90°)
            Shootingpoint1 = line(
                    p.of(59.601, 10.167, -90.0),
                    p.of(60.0, 25.0, -90.0)
            ).linear(p.of(59.601, 10.167, -90.0), p.of(60.0, 25.0, -90.0));

            // Intake Point #1: Drive to intake position facing (72, 72)
            IntakePoint1 = line(
                    p.of(60.0, 25.0, -90.0),
                    p.of(11.686, 12.037, 0.0)
            ).heading(
                    Interpolator.piecewise().until(
                            1,
                            Interpolator.facingPoint(p.of(72.000, 72.000, 0.0))
                    )
            );

            // Shooting Point #2: Curve to second shooting position
            ShootingPoint2 = curve(
                    p.of(11.686, 12.037, 0.0),
                    p.of(37.440, 103.658, 0.0),
                    p.of(24.233, 110.002, 0.0),
                    p.of(39.968, 114.176, 0.0)
            ).facingPoint(p.of(72.000, 72.000, 0.0));

            // Parking Point: Drive to final park spot
            ParkingPoint = line(
                    p.of(39.968, 114.176, 0.0),
                    p.of(10.050, 114.176, 0.0)
            ).facingPoint(p.of(72.000, 72.000, 0.0));
        }
    }

    @Override
    public void runOpMode() {
        // Initialize follower and hardware
        follower = Constants.create(hardwareMap);
        initHardware();

        // Set starting pose: (59.601, 10.167, -90°)
        follower.setPose(PoseFactory.degrees().of(59.601, 10.167, -90.0));
        follower.update();

        paths = new Paths();

        telemetry.addLine("TurtleTracerAuto Initialized & Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Spin up shooter & drive to Shootingpoint1
            spinUpShooter(SHOOT_VEL);
            telemetry.addLine("Navigating to Shootingpoint1 (First Shooting Spot)...");
            telemetry.update();
            followPath(paths.Shootingpoint1);

            // STEP 2: Auto-aim & shoot at First Shooting Spot
            telemetry.addLine("Arrived at Shootingpoint1! Auto-aiming and shooting...");
            telemetry.update();
            aimAndShootAtGoal(GOAL_X, GOAL_Y);

            // STEP 3: Turn on intake & drive along IntakePoint1 to collect rings
            telemetry.addLine("Intaking rings along IntakePoint1...");
            telemetry.update();
            intakeOn();
            followPath(paths.IntakePoint1);

            // STEP 4: Turn off intake, spin up shooter & drive along ShootingPoint2
            intakeOff();
            spinUpShooter(SHOOT_VEL);
            telemetry.addLine("Navigating to ShootingPoint2...");
            telemetry.update();
            followPath(paths.ShootingPoint2);

            // STEP 5: Auto-aim & shoot at Second Shooting Spot
            telemetry.addLine("Arrived at ShootingPoint2! Auto-aiming and shooting at Goal Target...");
            telemetry.update();
            aimAndShootAtGoal(GOAL_X, GOAL_Y);

            // STEP 6: Drive to final ParkingPoint
            telemetry.addLine("Navigating to ParkingPoint...");
            telemetry.update();
            followPath(paths.ParkingPoint);

            // STEP 7: Stop all subsystems
            stopAll();
            telemetry.addLine("Autonomous Route Complete!");
            telemetry.update();
        }

        stopAll();
    }

    // ── Navigation & Execution Helpers ──────────────────────────────────────

    public void followPath(Path path) {
        follower.follow(path);

        ElapsedTime startTimer = new ElapsedTime();
        while (opModeIsActive() && !follower.isBusy() && startTimer.milliseconds() < 100) {
            follower.update();
        }

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Robot X", "%.2f", follower.pose().x());
            telemetry.addData("Robot Y", "%.2f", follower.pose().y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
            if (follower.currentPath() != null) {
                telemetry.addData("Remaining Distance", "%.2f in", follower.distanceToEndpoint());
            }
            telemetry.update();
        }
    }

    public void waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            follower.update();
        }
    }

    // ── Subsystem Control Methods (From TeleOps) ─────────────────────────────

    public void spinUpShooter(double targetVelocity) {
        if (leftShooter != null && rightShooter != null) {
            leftShooter.setVelocity(targetVelocity);
            rightShooter.setVelocity(targetVelocity);
        }
    }

    public void stopShooter() {
        if (leftShooter != null && rightShooter != null) {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);
        }
    }

    public void intakeOn() {
        if (leftIntake != null && rightIntake != null) {
            leftIntake.setPower(INTAKE_POWER);
            rightIntake.setPower(INTAKE_POWER);
        }
    }

    public void intakeOff() {
        if (leftIntake != null && rightIntake != null) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void shootRings() {
        shootRings(SPINUP_SETTLE_MS, KICK_DELAY_MS, SHOOT_HOLD_MS);
    }

    public void shootRings(long settleMs, long kickDelayMs, long holdMs) {
        spinUpShooter(SHOOT_VEL);
        waitMs(settleMs);

        if (hood != null) hood.setPosition(HOOD_LOWEST);
        if (blocker != null) blocker.setPosition(BLOCKER_OPEN);
        intakeOn();
        if (kicker != null) kicker.setPosition(KICKER_BASE);

        waitMs(kickDelayMs);
        if (kicker != null) kicker.setPosition(KICKER_KICK);
        waitMs(holdMs);

        if (kicker != null) kicker.setPosition(KICKER_BASE);
        if (blocker != null) blocker.setPosition(BLOCKER_CLOSED);
        if (hood != null) hood.setPosition(HOOD_START);
        intakeOff();
        stopShooter();
    }

    public void aimAndShootAtGoal(double goalX, double goalY) {
        intakeOff();
        Pose currentPose = follower.pose();
        double dx = goalX - currentPose.x();
        double dy = goalY - currentPose.y();
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Rotate in place to face goal target
        Pose aimedPose = PoseFactory.degrees().of(currentPose.x(), currentPose.y(), targetHeadingDeg);
        followPath(line(currentPose, aimedPose).linear(currentPose, aimedPose));

        // Execute shooting sequence
        shootRings(300, KICK_DELAY_MS, SHOOT_HOLD_MS);
    }

    // ── Hardware Initialization & Cleanup ────────────────────────────────────

    private void initHardware() {
        // Configure Shooter Motors
        try {
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
        } catch (Exception ignored) {}

        // Configure Intake Motors
        try {
            leftIntake  = hardwareMap.get(DcMotorEx.class, "li");
            rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
            leftIntake.setDirection(INTAKE_FLIPPED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rightIntake.setDirection(INTAKE_FLIPPED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {}

        // Configure Servos
        try {
            blocker = hardwareMap.get(Servo.class, "blocker");
            kicker  = hardwareMap.get(Servo.class, "kicker");
            hood    = hardwareMap.get(Servo.class, "hood");

            blocker.setPosition(BLOCKER_CLOSED);
            kicker.setPosition(KICKER_BASE);
            hood.setPosition(HOOD_START);
        } catch (Exception ignored) {}
    }

    private void stopAll() {
        stopShooter();
        intakeOff();
        if (blocker != null) blocker.setPosition(BLOCKER_CLOSED);
        if (kicker != null) kicker.setPosition(KICKER_BASE);
        if (hood != null) hood.setPosition(HOOD_START);
    }
}