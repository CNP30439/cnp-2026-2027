package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

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
//  AutoPathShoot — Autonomous Opmode (FTC BioBuzz Season)
//
//  SEQUENCE:
//    1. Initialize hardware, set starting pose, spin up flywheels.
//    2. Drive Path 1 -> First shooting location (point1).
//    3. Execute shooting sequence (shootRings).
//    4. Turn on intake while driving Path 2 -> Waypoint (avoid).
//    5. Drive Path 3 -> Final parking/substructure spot (point6).
//    6. Stop all motors and servos.
//
//  HARDWARE NAMES (Driver Hub):
//    Drive Motors : Handled via pedro/Constants.java (fl, fr, bl, br)
//    Shooter      : "ls" (left), "rs" (right)
//    Intake       : "li" (left), "ri" (right)
//    Servos       : "blocker", "kicker", "hood"
// ═══════════════════════════════════════════════════════════════════════════════
@Autonomous(name = "AutoPathShoot", group = "Autonomous")
public class AutoPathShoot extends LinearOpMode {

    private Follower follower;

    // ── Hardware Declarations ────────────────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx leftIntake, rightIntake;
    private Servo blocker, kicker, hood;

    // ── Shooter Settings (PIDF & Velocities) ─────────────────────────────────
    private static final double SHOOTER_P       = 1.5;
    private static final double SHOOTER_F_LEFT  = 18.3237;
    private static final double SHOOTER_F_RIGHT = 18.3237;

    // Target velocity in ticks/sec (matches TeleOp tuned speed)
    private static final double SHOOT_VEL = 1250;

    // ── Servo Positions & Hardware Parameters ────────────────────────────────
    private static final boolean USE_HOOD   = true;
    private static final double HOOD_START  = 0.5;
    private static final double HOOD_LOWEST = 0.35;

    private static final boolean INTAKE_FLIPPED = true;
    private static final double INTAKE_POWER    = 1.0;

    private static final double BLOCKER_OPEN   = 0.6;
    private static final double BLOCKER_CLOSED = 1.0;

    private static final double KICKER_BASE = 0.8;
    private static final double KICKER_KICK = 0.5;

    // ── Action Delays (ms) ───────────────────────────────────────────────────
    private static final long SPINUP_SETTLE_MS = 5000;  // 5 seconds spinup / load-up time before shooting
    private static final long KICK_DELAY_MS    = 500;   // Time for intake to feed ring to kicker
    private static final long SHOOT_HOLD_MS    = 1500;  // Time kicker holds forward to clear rings

    // ── Goal Endpoint Coordinates & Orientation Offset ───────────────────────
    private static final double GOAL_X = 62.68523316062177;
    private static final double GOAL_Y = 86.5373935468868;
    public static final double SHOOTER_HEADING_OFFSET_DEG = 180.0;

    // ── Field Poses & Coordinates (From Visualizer / User Spec) ──────────────
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start        = poseFactory.of(32, 9, -90);
    private final Pose point1Start  = poseFactory.of(32, 9, -90);
    private final Pose point1       = poseFactory.of(60, 25, -90);
    private final Pose point2       = poseFactory.of(15.5813, 8.8872, -90);
    private final Pose point3       = poseFactory.of(6.6084, 8.535, -120);
    private final Pose point4       = poseFactory.of(31.535, 23.0358, -120);
    private final Pose point5       = poseFactory.of(32.9187, 128.6434, 120);
    private final Pose point6       = poseFactory.of(17.1976, 119.7692, 120);

    @Override
    public void runOpMode() {
        // Initialize follower and hardware
        follower = Constants.create(hardwareMap);
        initHardware();

        follower.setPose(start);
        follower.update();

        telemetry.addLine("Initialized and ready for FTC BioBuzz Autonomous.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Spin up shooter flywheels while starting move
            spinUpShooter(SHOOT_VEL);

            // STEP 2: Drive Path 1 to the first shooting location
            telemetry.addLine("Navigating Path 1 -> First Shooting Spot (point1)...");
            telemetry.update();
            followPath(path1());

            // STEP 3: Load up flywheels for 5 seconds before shooting rings
            telemetry.addLine("Arrived at point1. Loading up flywheels for 5 seconds before shooting...");
            telemetry.update();
            shootRings();

            // STEP 4: Start intake & follow paths 2 through 5
            intakeOn();

            telemetry.addLine("Following Path 2...");
            telemetry.update();
            followPath(path2());

            telemetry.addLine("Following Path 3...");
            telemetry.update();
            followPath(path3());

            telemetry.addLine("Following Path 4...");
            telemetry.update();
            followPath(path4());

            telemetry.addLine("Following Path 5 -> Second Shooting Spot (point5)...");
            telemetry.update();
            followPath(path5());

            // STEP 5: Aim at Goal Endpoint (62.685, 86.537) and shoot before Path 6
            telemetry.addLine("Aiming and shooting at Goal Endpoint...");
            telemetry.update();
            aimAndShootAtGoal(GOAL_X, GOAL_Y);

            // STEP 6: Drive Path 6 -> Final Spot (point6)
            telemetry.addLine("Following Path 6 -> Final Spot...");
            telemetry.update();
            followPath(path6());

            // STEP 7: Turn off intake and stop all subsystems
            intakeOff();
            stopAll();

            telemetry.addLine("Autonomous Route Complete!");
            telemetry.update();
        }

        stopAll();
    }

    // ── Path Definitions ─────────────────────────────────────────────────────

    public Path path1() {
        return Paths.line(point1Start, point1).linear(point1Start, point1);
    }

    public Path path2() {
        return Paths.line(point1, point2).linear(point1, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).linear(point3, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).linear(point4, point5);
    }

    public Path path6() {
        Pose currentPose = follower.pose();
        return Paths.line(currentPose, point6).linear(currentPose, point6);
    }

    // ── Execution Helpers ────────────────────────────────────────────────────

    /**
     * Follows a Pedro Path reliably without race conditions or skipping.
     * Continuously calls follower.update() while monitoring active execution.
     */
    public void followPath(Path path) {
        follower.follow(path);

        // Brief delay loop ensuring follower registers busy state
        ElapsedTime startTimer = new ElapsedTime();
        while (opModeIsActive() && !follower.isBusy() && startTimer.milliseconds() < 100) {
            follower.update();
        }

        // Keep updating until robot arrives at target pose
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

    /** Wait for specified milliseconds without freezing Pedro Pathing updates. */
    public void waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            follower.update();
        }
    }

    // ── Shooter Controls ─────────────────────────────────────────────────────

    public void spinUpShooter(double targetVelocity) {
        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);
    }

    public void stopShooter() {
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    /**
     * Full sequence to feed and shoot rings:
     * 1. Stabilize flywheels & drop hood
     * 2. Open blocker & run intake to feed
     * 3. Trigger kicker to push rings through flywheel
     * 4. Reset servos and intake
     */
    public void shootRings() {
        shootRings(SPINUP_SETTLE_MS, KICK_DELAY_MS, SHOOT_HOLD_MS);
    }

    public void shootRings(long settleMs, long kickDelayMs, long holdMs) {
        spinUpShooter(SHOOT_VEL);
        waitMs(settleMs);

        // Drop hood for shot, open blocker, run intake to feed
        if (USE_HOOD && hood != null) hood.setPosition(HOOD_LOWEST);
        blocker.setPosition(BLOCKER_OPEN);
        intakeOn();
        kicker.setPosition(KICKER_BASE);

        // Wait for ring to feed, then trigger kicker
        waitMs(kickDelayMs);
        kicker.setPosition(KICKER_KICK);
        waitMs(holdMs);

        // Reset shooter mechanisms
        kicker.setPosition(KICKER_BASE);
        blocker.setPosition(BLOCKER_CLOSED);
        if (USE_HOOD && hood != null) hood.setPosition(HOOD_START);
        intakeOff();
        stopShooter();
    }

    /**
     * Aim at the specified Goal Endpoint coordinates (goalX, goalY), rotate in place, and execute shot.
     */
    public void aimAndShootAtGoal(double goalX, double goalY) {
        intakeOff();
        Pose currentPose = follower.pose();
        double dx = goalX - currentPose.x();
        double dy = goalY - currentPose.y();

        // Calculate heading with shooter 180° offset
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx)) + SHOOTER_HEADING_OFFSET_DEG;
        while (targetHeadingDeg > 180)  targetHeadingDeg -= 360;
        while (targetHeadingDeg < -180) targetHeadingDeg += 360;

        // Rotate in place to face goal target with shooter
        Pose aimedPose = poseFactory.of(currentPose.x(), currentPose.y(), targetHeadingDeg);
        followPath(Paths.line(currentPose, aimedPose).linear(currentPose, aimedPose));

        // Execute shot sequence
        shootRings(300, KICK_DELAY_MS, SHOOT_HOLD_MS);
    }

    // ── Intake Controls ──────────────────────────────────────────────────────

    public void intakeOn() {
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);
    }

    public void intakeReverse() {
        leftIntake.setPower(-INTAKE_POWER);
        rightIntake.setPower(-INTAKE_POWER);
    }

    public void intakeOff() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void runIntakeFor(long durationMs) {
        intakeOn();
        waitMs(durationMs);
        intakeOff();
    }

    // ── Servo Mechanism Direct Controls ──────────────────────────────────────

    public void setHoodPosition(double position) {
        if (USE_HOOD && hood != null) {
            hood.setPosition(position);
        }
    }

    public void setBlockerOpen(boolean open) {
        blocker.setPosition(open ? BLOCKER_OPEN : BLOCKER_CLOSED);
    }

    public void setKickerFired(boolean kick) {
        kicker.setPosition(kick ? KICKER_KICK : KICKER_BASE);
    }

    // ── Hardware Initialization & Cleanup ────────────────────────────────────

    private void initHardware() {
        // Configure Shooter Motors
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

        // Configure Intake Motors
        leftIntake  = hardwareMap.get(DcMotorEx.class, "li");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
        leftIntake.setDirection(INTAKE_FLIPPED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(INTAKE_FLIPPED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure Servos
        blocker = hardwareMap.get(Servo.class, "blocker");
        kicker  = hardwareMap.get(Servo.class, "kicker");
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);

        if (USE_HOOD) {
            hood = hardwareMap.get(Servo.class, "hood");
            hood.setPosition(HOOD_START);
        }
    }

    private void stopAll() {
        stopShooter();
        intakeOff();
        blocker.setPosition(BLOCKER_CLOSED);
        kicker.setPosition(KICKER_BASE);
        if (USE_HOOD && hood != null) hood.setPosition(HOOD_START);
    }
}