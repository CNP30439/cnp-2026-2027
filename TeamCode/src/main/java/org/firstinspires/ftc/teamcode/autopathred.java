package org.firstinspires.ftc.teamcode;
//hi
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class autopathred extends OpMode {

    // ---- TUNING CONSTANTS (placeholders: set these to your robot's real values) ----
    private static final double SHOOTER_HEADING_OFFSET_DEG = 0.0;
    private static final double SHOOT_VEL = 1500;
    private static final double SHOOTER_P = 10.0;
    private static final double SHOOTER_F_LEFT = 12.0;
    private static final double SHOOTER_F_RIGHT = 12.0;
    private static final double INTAKE_POWER = 1.0;
    private static final boolean INTAKE_FLIPPED = false;
    private static final double HOOD_START = 0.0;
    private static final double HOOD_LOWEST = 0.0;
    private static final double BLOCKER_OPEN = 0.5;
    private static final double BLOCKER_CLOSED = 0.0;
    private static final double KICKER_BASE = 0.0;

    // ---- Hardware ----
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx leftIntake, rightIntake;
    private Servo blocker, kicker, hood;

    public Follower follower; // Pathing follower instance
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private ElapsedTime shootTimer; // Timer for shooting sequence
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();

        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(60.000, 9.808, -90.000));

        pathTimer = new ElapsedTime();
        shootTimer = new ElapsedTime();
        paths = new Paths(follower); // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update follower
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
    }

    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path DriveToShoot;
        public Path line2;
        public Path line3;
        public Path line4;
        public Path line5;
        public Path line6;

        public Paths(Follower follower) {
            DriveToShoot = line(
                    p.of(60.000, 9.808, 0.0),
                    p.of(60.000, 25.000, 0.0)
            ).linear(Math.toRadians(-90), Math.toRadians(-90));

            line2 = curve(
                    p.of(60.000, 25.000, 0.0),
                    p.of(54.476, 16.544, 0.0),
                    p.of(11.807, 10.000, 0.0)
            ).linear(Math.toRadians(-90), Math.toRadians(180));

            line3 = curve(
                    p.of(11.807, 10.000, 0.0),
                    p.of(38.352, 42.932, 0.0),
                    p.of(18.666, 120.904, 0.0),
                    p.of(60.000, 117.770, 0.0)
            ).linear(Math.toRadians(180), Math.toRadians(90));

            line4 = line(
                    p.of(60.000, 117.770, 0.0),
                    p.of(49.000, 128.312, 0.0)
            ).linear(Math.toRadians(90), Math.toRadians(90));

            line5 = line(
                    p.of(49.000, 128.312, 0.0),
                    p.of(60.000, 117.770, 0.0)
            ).linear(Math.toRadians(90), Math.toRadians(90));

            line6 = line(
                    p.of(60.000, 117.770, 0.0),
                    p.of(11.807, 108.134, 0.0)
            ).linear(Math.toRadians(90), Math.toRadians(90));
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(paths.DriveToShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.follow(paths.line2);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.follow(paths.line3);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.follow(paths.line4);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.follow(paths.line5);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.follow(paths.line6);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
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

    // ================= Shooter / intake methods =================

    public void aimAtGoal(double goalX, double goalY) {
        intakeOff();
        Pose currentPose = follower.pose();
        double dx = goalX - currentPose.x();
        double dy = goalY - currentPose.y();

        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx)) + SHOOTER_HEADING_OFFSET_DEG;
        while (targetHeadingDeg > 180)  targetHeadingDeg -= 360;
        while (targetHeadingDeg < -180) targetHeadingDeg += 360;

        Pose aimedPose = p.of(currentPose.x(), currentPose.y(), targetHeadingDeg);
        // Same signature as the other paths: linear(startHeadingRadians, endHeadingRadians)
        follower.follow(line(currentPose, aimedPose)
                .linear(currentPose.heading(), Math.toRadians(targetHeadingDeg)));
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