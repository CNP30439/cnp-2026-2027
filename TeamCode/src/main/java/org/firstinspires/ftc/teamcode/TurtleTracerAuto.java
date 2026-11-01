package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// ═══════════════════════════════════════════════════════════════════════════════
//  TurtleTracerAuto — Iterative OpMode Autonomous (FTC BioBuzz Season)
//  Auto-generated paths with reverse interpolators & shooting/intaking state machine
// ═══════════════════════════════════════════════════════════════════════════════
@Autonomous(name = "AutoPath", group = "Autonomous")
public class TurtleTracerAuto extends OpMode {

    public Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState;
    private ElapsedTime pathTimer;
    private ElapsedTime shootTimer;
    private Paths paths;

    // ── High Goal Coordinates & Orientation Offset ───────────────────────────
    public static final double GOAL_X = 62.6852;
    public static final double GOAL_Y = 86.5374;
    public static final double SHOOTER_HEADING_OFFSET_DEG = 180.0;

    // ── Subsystem Hardware Declarations ──────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx leftIntake, rightIntake;
    private Servo blocker, kicker, hood;

    // ── Subsystem Parameters & Constants ─────────────────────────────────────
    private static final double SHOOTER_P       = 1.5;
    private static final double SHOOTER_F_LEFT  = 18.3237;
    private static final double SHOOTER_F_RIGHT = 18.3237;
    private static final double SHOOT_VEL       = 1250;

    private static final boolean INTAKE_FLIPPED = true;
    private static final double INTAKE_POWER    = 1.0;

    private static final double BLOCKER_OPEN   = 0.6;
    private static final double BLOCKER_CLOSED = 1.0;

    private static final double KICKER_BASE = 0.8;
    private static final double KICKER_KICK = 0.5;

    private static final double HOOD_START  = 0.5;
    private static final double HOOD_LOWEST = 0.35;

    private static final long KICK_DELAY_MS = 500;
    private static final long SHOOT_HOLD_MS = 1500;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(59.601, 10.167, -90.000));

        initHardware();

        pathTimer  = new ElapsedTime();
        shootTimer = new ElapsedTime();
        paths      = new Paths(follower);

        telemetry.addLine("TurtleTracerAuto Initialized & Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    // ── Turtle Tracer Paths Definition ───────────────────────────────────────
    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path Shootingpoint1;
        public Path IntakePoint1;
        public Path ShootingPoint2;
        public Path ParkingPoint;

        public Paths(Follower follower) {
            this.Shootingpoint1 = line(
                    p.of(59.601, 10.167, 0.0),
                    p.of(59.601, 22.087, 0.0)
            ).reverseTangent();

            IntakePoint1 = line(
                    p.of(59.601, 22.087, 0.0),
                    p.of(11.686, 12.037, 0.0)
            ).heading(
                    Interpolator.piecewise().until(
                            1,
                            Interpolator.facingPoint(p.of(72.000, 72.000, 0.0)).reverse()
                    )
            );

            ShootingPoint2 = curve(
                    p.of(11.686, 12.037, 0.0),
                    p.of(37.440, 103.658, 0.0),
                    p.of(24.233, 110.002, 0.0),
                    p.of(39.968, 114.176, 0.0)
            ).heading(Interpolator.facingPoint(p.of(72.000, 72.000, 0.0)).reverse());

            ParkingPoint = line(
                    p.of(39.968, 114.176, 0.0),
                    p.of(10.050, 114.176, 0.0)
            ).heading(Interpolator.facingPoint(p.of(72.000, 72.000, 0.0)).reverse());
        }
    }

    // ── Non-Blocking Autonomous State Machine ────────────────────────────────
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start drive to Shootingpoint1 & spin up flywheels
                spinUpShooter(SHOOT_VEL);
                follower.follow(paths.Shootingpoint1);
                setPathState(1);
                break;

            case 1:
                // Arrived at Shootingpoint1 -> Start Auto-Aim at Goal
                if (!follower.isBusy()) {
                    aimAtGoal(GOAL_X, GOAL_Y);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for Auto-Aim rotation to complete
                if (!follower.isBusy()) {
                    startShootingSequence();
                    setPathState(11);
                }
                break;

            case 11:
                // Execute Shooting Sequence timer
                if (shootTimer.milliseconds() >= KICK_DELAY_MS && kicker != null) {
                    kicker.setPosition(KICKER_KICK);
                }
                if (shootTimer.milliseconds() >= (KICK_DELAY_MS + SHOOT_HOLD_MS)) {
                    resetShootingMechanism();

                    // Start drive to IntakePoint1 with intake ON
                    intakeOn();
                    follower.follow(paths.IntakePoint1);
                    setPathState(2);
                }
                break;

            case 2:
                // Driving & intaking along IntakePoint1
                intakeOn();
                if (!follower.isBusy()) {
                    intakeOff();
                    spinUpShooter(SHOOT_VEL);

                    // Drive to ShootingPoint2
                    follower.follow(paths.ShootingPoint2);
                    setPathState(3);
                }
                break;

            case 3:
                // Arrived at ShootingPoint2 -> Start Auto-Aim at Goal
                if (!follower.isBusy()) {
                    aimAtGoal(GOAL_X, GOAL_Y);
                    setPathState(20);
                }
                break;

            case 20:
                // Wait for Auto-Aim rotation to complete
                if (!follower.isBusy()) {
                    startShootingSequence();
                    setPathState(21);
                }
                break;

            case 21:
                // Execute Shooting Sequence #2 timer
                if (shootTimer.milliseconds() >= KICK_DELAY_MS && kicker != null) {
                    kicker.setPosition(KICKER_KICK);
                }
                if (shootTimer.milliseconds() >= (KICK_DELAY_MS + SHOOT_HOLD_MS)) {
                    resetShootingMechanism();

                    // Drive to final ParkingPoint
                    follower.follow(paths.ParkingPoint);
                    setPathState(4);
                }
                break;

            case 4:
                // Arrived at ParkingPoint -> Stop Autonomous
                if (!follower.isBusy()) {
                    stopAll();
                    setPathState(5);
                }
                break;

            case 5:
                stopAll();
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    // ── Action & Subsystem Helpers ───────────────────────────────────────────

    public void aimAtGoal(double goalX, double goalY) {
        intakeOff();
        Pose currentPose = follower.pose();
        double dx = goalX - currentPose.x();
        double dy = goalY - currentPose.y();

        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx)) + SHOOTER_HEADING_OFFSET_DEG;
        while (targetHeadingDeg > 180)  targetHeadingDeg -= 360;
        while (targetHeadingDeg < -180) targetHeadingDeg += 360;

        Pose aimedPose = p.of(currentPose.x(), currentPose.y(), targetHeadingDeg);
        follower.follow(line(currentPose, aimedPose).linear(currentPose, aimedPose));
    }

    public void startShootingSequence() {
        spinUpShooter(SHOOT_VEL);
        if (hood != null) hood.setPosition(HOOD_LOWEST);
        if (blocker != null) blocker.setPosition(BLOCKER_OPEN);
        intakeOn();
        if (kicker != null) kicker.setPosition(KICKER_BASE);
        shootTimer.reset();
    }

    public void resetShootingMechanism() {
        if (kicker != null) kicker.setPosition(KICKER_BASE);
        if (blocker != null) blocker.setPosition(BLOCKER_CLOSED);
        if (hood != null) hood.setPosition(HOOD_START);
        intakeOff();
        stopShooter();
    }

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