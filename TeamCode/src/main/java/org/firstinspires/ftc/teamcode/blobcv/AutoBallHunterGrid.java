package org.firstinspires.ftc.teamcode.blobcv;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "AutoBallHunterGrid", group = "Autonomous")
public class AutoBallHunterGrid extends LinearOpMode {

    // ── Two Camera Modes (Logitech or Limelight 3A) ──────────────────────────
    public enum CameraMode {
        LOGITECH_WEBCAM,   // Mode 1: Logitech RGB Webcam
        LIMELIGHT_3A       // Mode 2: Limelight 3A Sensor
    }
    private CameraMode selectedCameraMode = CameraMode.LOGITECH_WEBCAM;

    private static final int IMAGE_WIDTH  = 320;
    private static final int IMAGE_HEIGHT = 240;
    private static final double TARGET_X  = IMAGE_WIDTH / 2.0;

    private static final double TARGET_AREA = 3000;
    private static final double TARGET_TA   = 8.0;   // Limelight target area percentage
    private static final int BALL_PIPELINE   = 8;     // Limelight color pipeline index
    private static final double TILE_SIZE_INCHES = 0.5;

    // ── Goal Target Coordinates for Autonomous Shooting ──────────────────────
    public static final double GOAL_X = 62.6852;
    public static final double GOAL_Y = 86.5374;
    public static final double SHOOTER_HEADING_OFFSET_DEG = 180.0;

    // ── Hardware Declarations (From TeleOps) ─────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx leftIntake, rightIntake;
    private Servo blocker, kicker, hood;
    private DistanceSensor ballSensor;

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

    private static final long KICK_DELAY_MS = 500;
    private static final long SHOOT_HOLD_MS = 1500;

    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91},
            {91, 50, 96, 91}
    };
    private static final double ROBOT_MARGIN = 7.0;
    private static final double CORNER_RADIUS = 10.0;
    private static final int    CURVE_SAMPLES = 20;
    private static final double PAD_STEP      = 1.5;
    private static final int    PAD_TRIES     = 10;
    private static final double FIELD_MIN     = 0;
    private static final double FIELD_MAX     = 144;
    private static final double eps           = 1e-5;

    private final PoseFactory pFactory = PoseFactory.degrees();

    private Follower follower;

    private final Map<String, TrackedTile> spatialTileMap = new HashMap<>();
    private String currentLockedTargetTileKey = null;

    private VisionPortal webcamPortal = null;
    private BrightBlobProcessor webcamProcessor = null;
    private Limelight3A limelight = null;

    // ── FSM States for Full Collection & Shooting Workflow ─────────────────
    private enum AutoState {
        SEARCH_SPIN,
        NAVIGATING_TO_BALL,
        INTAKING_BALL,
        NAVIGATING_TO_SHOOT,
        SHOOTING
    }
    private AutoState currentAutoState = AutoState.SEARCH_SPIN;

    private static class TrackedTile {
        int tileX;
        int tileY;
        boolean hasBall;
        long lastSeenTimestamp;

        TrackedTile(int x, int y) {
            this.tileX = x;
            this.tileY = y;
            this.hasBall = true;
            this.lastSeenTimestamp = System.currentTimeMillis();
        }
    }

    static class BrightBlobProcessor implements VisionProcessor {

        public double centerX = 0;
        public double centerY = 0;
        public double area    = 0;

        private final int lowThresh;
        private final int highThresh;

        public BrightBlobProcessor(int lowThresh, int highThresh) {
            this.lowThresh  = lowThresh;
            this.highThresh = highThresh;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat mask = new Mat();
            Imgproc.threshold(frame, mask, lowThresh, highThresh, Imgproc.THRESH_BINARY);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(),
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint bestContour = null;
            double maxArea = 0;

            for (MatOfPoint c : contours) {
                double a = Imgproc.contourArea(c);
                if (a > maxArea) {
                    maxArea = a;
                    bestContour = c;
                }
            }

            if (bestContour != null) {
                Rect box = Imgproc.boundingRect(bestContour);
                centerX = box.x + box.width / 2.0;
                centerY = box.y + box.height / 2.0;
                area    = maxArea;
            } else {
                centerX = centerY = area = 0;
            }

            mask.release();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onDrawFrameWidth, int onDrawFrameHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        initSubsystems();

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(109.5, 132, Math.toRadians(90)));
        follower.update();

        // ── Camera Mode Selection Phase in INIT ──────────────────────────────
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up || gamepad1.x || gamepad1.dpad_left || gamepad1.a) {
                selectedCameraMode = CameraMode.LOGITECH_WEBCAM;
            } else if (gamepad1.dpad_down || gamepad1.y || gamepad1.dpad_right || gamepad1.b) {
                selectedCameraMode = CameraMode.LIMELIGHT_3A;
            }

            telemetry.addLine("=== SELECT CAMERA MODE (INIT) ===");
            telemetry.addLine("Gamepad 1 DPAD UP / X / A  -> 1. Logitech RGB Webcam");
            telemetry.addLine("Gamepad 1 DPAD DOWN / Y / B -> 2. Limelight 3A Sensor");
            telemetry.addLine("------------------------------------");
            telemetry.addData("SELECTED CAMERA MODE", selectedCameraMode);
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        // ── Initialize Vision System Based on Selected Mode ──────────────────
        if (selectedCameraMode == CameraMode.LOGITECH_WEBCAM) {
            webcamProcessor = new BrightBlobProcessor(170, 255);
            webcamPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(webcamProcessor)
                    .build();

            while (!isStopRequested() && webcamPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            lockCameraSettings(webcamPortal);

        } else if (selectedCameraMode == CameraMode.LIMELIGHT_3A) {
            limelight = hardwareMap.get(Limelight3A.class, "orange");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(BALL_PIPELINE);
            limelight.start();
        }

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose robotPose = follower.pose();
            double robotGlobalX = robotPose.x();
            double robotGlobalY = robotPose.y();
            double robotGlobalHeadingRad = robotPose.heading();

            double ballGlobalX = 0;
            double ballGlobalY = 0;
            boolean detectedBall = false;

            // ── Camera Frame Processing ──────────────────────────────────────
            if (selectedCameraMode == CameraMode.LOGITECH_WEBCAM && webcamProcessor != null) {
                double rawArea = webcamProcessor.area;
                if (rawArea > 0) {
                    double centerImgX = webcamProcessor.centerX;
                    double estimatedDistanceInches = Math.sqrt(TARGET_AREA / Math.max(1, rawArea)) * 12.0;
                    double angleOffsetRad = ((centerImgX - TARGET_X) / IMAGE_WIDTH) * Math.toRadians(60.0);
                    double absoluteAngleRad = robotGlobalHeadingRad + angleOffsetRad;

                    ballGlobalX = robotGlobalX + (estimatedDistanceInches * Math.cos(absoluteAngleRad));
                    ballGlobalY = robotGlobalY + (estimatedDistanceInches * Math.sin(absoluteAngleRad));
                    detectedBall = true;
                }
            } else if (selectedCameraMode == CameraMode.LIMELIGHT_3A && limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ta = result.getTa();
                    if (ta > 0.1) {
                        double estimatedDistanceInches = Math.sqrt(TARGET_TA / Math.max(0.01, ta)) * 12.0;
                        double angleOffsetRad = Math.toRadians(tx);
                        double absoluteAngleRad = robotGlobalHeadingRad + angleOffsetRad;

                        ballGlobalX = robotGlobalX + (estimatedDistanceInches * Math.cos(absoluteAngleRad));
                        ballGlobalY = robotGlobalY + (estimatedDistanceInches * Math.sin(absoluteAngleRad));
                        detectedBall = true;
                    }
                }
            }

            // ── Update Spatial Tile Grid Map ─────────────────────────────────
            if (detectedBall) {
                int tileX = (int) Math.floor(ballGlobalX / TILE_SIZE_INCHES);
                int tileY = (int) Math.floor(ballGlobalY / TILE_SIZE_INCHES);
                String discoveredTileKey = tileX + "," + tileY;

                if (!spatialTileMap.containsKey(discoveredTileKey)) {
                    spatialTileMap.put(discoveredTileKey, new TrackedTile(tileX, tileY));
                } else {
                    spatialTileMap.get(discoveredTileKey).lastSeenTimestamp = System.currentTimeMillis();
                    spatialTileMap.get(discoveredTileKey).hasBall = true;
                }
            }

            // ── Autonomous FSM: Search -> Intake -> Aim & Shoot ─────────────
            switch (currentAutoState) {

                case SEARCH_SPIN:
                    intakeOff();
                    stopShooter();
                    String closestTargetKey = findNearestKnownFilledTile(robotGlobalX, robotGlobalY);

                    if (closestTargetKey != null) {
                        currentLockedTargetTileKey = closestTargetKey;
                        String[] coords = closestTargetKey.split(",");
                        double targetFieldX = (Integer.parseInt(coords[0]) * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);
                        double targetFieldY = (Integer.parseInt(coords[1]) * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);
                        Pose targetPose = new Pose(targetFieldX, targetFieldY, robotGlobalHeadingRad);

                        drive(0, 0, 0);
                        intakeOn(); // Turn on intake while driving to collect ball
                        scheduleRoute(robotPose, targetPose);
                        currentAutoState = AutoState.NAVIGATING_TO_BALL;
                    } else {
                        drive(0, 0, 0.25); // Spin to search
                    }
                    break;

                case NAVIGATING_TO_BALL:
                    intakeOn();
                    if (!follower.isBusy()) {
                        Scheduler.reset();
                        currentAutoState = AutoState.INTAKING_BALL;
                    }
                    break;

                case INTAKING_BALL:
                    intakeOn();
                    waitMs(500); // Wait briefly to pull ball fully into intake
                    intakeOff();

                    if (currentLockedTargetTileKey != null && spatialTileMap.containsKey(currentLockedTargetTileKey)) {
                        spatialTileMap.get(currentLockedTargetTileKey).hasBall = false;
                        currentLockedTargetTileKey = null;
                    }

                    // Spin up flywheels for shooting
                    spinUpShooter(SHOOT_VEL);
                    currentAutoState = AutoState.NAVIGATING_TO_SHOOT;
                    break;

                case NAVIGATING_TO_SHOOT:
                    // Rotate in place to face High Goal
                    aimAndShootAtGoal(GOAL_X, GOAL_Y);
                    currentAutoState = AutoState.SEARCH_SPIN;
                    break;

                case SHOOTING:
                    currentAutoState = AutoState.SEARCH_SPIN;
                    break;
            }

            telemetry.addData("Selected Camera Mode", selectedCameraMode);
            telemetry.addData("Auto Execution Mode", currentAutoState);
            telemetry.addData("Robot Field Pose", "X: %.1f, Y: %.1f", robotGlobalX, robotGlobalY);
            telemetry.update();
        }

        stopAll();
        if (webcamPortal != null) webcamPortal.close();
        if (limelight != null) limelight.stop();
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

        // Calculate heading with shooter 180° offset
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx)) + SHOOTER_HEADING_OFFSET_DEG;
        while (targetHeadingDeg > 180)  targetHeadingDeg -= 360;
        while (targetHeadingDeg < -180) targetHeadingDeg += 360;

        // Rotate in place to face goal target with shooter
        Pose aimedPose = pFactory.of(currentPose.x(), currentPose.y(), targetHeadingDeg);
        scheduleRoute(currentPose, aimedPose);

        // Wait until rotation completes
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !follower.isBusy() && timer.milliseconds() < 100) {
            follower.update();
        }
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // Execute shooting sequence
        shootRings(300, KICK_DELAY_MS, SHOOT_HOLD_MS);
    }

    public void waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            follower.update();
        }
    }

    // ── Subsystem Hardware Initialization ────────────────────────────────────

    private void initSubsystems() {
        // Drive Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Shooter Motors
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

        // Intake Motors
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

        // Servos
        try {
            blocker = hardwareMap.get(Servo.class, "blocker");
            kicker  = hardwareMap.get(Servo.class, "kicker");
            hood    = hardwareMap.get(Servo.class, "hood");

            blocker.setPosition(BLOCKER_CLOSED);
            kicker.setPosition(KICKER_BASE);
            hood.setPosition(HOOD_START);
        } catch (Exception ignored) {}

        // Distance Sensor
        try {
            ballSensor = hardwareMap.get(DistanceSensor.class, "distance");
        } catch (Exception ignored) {}
    }

    private void stopAll() {
        drive(0, 0, 0);
        stopShooter();
        intakeOff();
        if (blocker != null) blocker.setPosition(BLOCKER_CLOSED);
        if (kicker != null) kicker.setPosition(KICKER_BASE);
        if (hood != null) hood.setPosition(HOOD_START);
    }

    // ── Route & Path Planning ────────────────────────────────────────────────

    private void scheduleRoute(Pose start, Pose target) {
        if (isBlockedLine(start.x(), start.y(), target.x(), target.y())) {
            Path curvedRoute = buildCurvedPath(start, target);
            if (curvedRoute != null) {
                schedule(follow(follower, curvedRoute));
                return;
            }
        }
        schedule(follow(follower, line(start, target).linear(start, target)));
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
            if (!segmentsClear(segs)) continue;
            double h0 = start.heading();
            double delta = Math.atan2(Math.sin(target.heading() - h0), Math.cos(target.heading() - h0));
            double total = 0;
            for (double[][] sg : segs) total += controlLength(sg);
            List parts = new ArrayList<>();
            double run = 0;
            for (double[][] sg : segs) {
                double len = controlLength(sg);
                double hA = h0 + delta * (run / total);
                double hB = h0 + delta * ((run + len) / total);
                run += len;
                Pose a = pFactory.of(sg[0][0], sg[0][1], Math.toDegrees(hA));
                Pose b = pFactory.of(sg[sg.length - 1][0], sg[sg.length - 1][1], Math.toDegrees(hB));
                if (sg.length == 2) {
                    parts.add(line(a, b).linear(a, b));
                } else {
                    Pose m = pFactory.of(sg[1][0], sg[1][1], Math.toDegrees(hA));
                    parts.add(curve(a, m, b).linear(a, b));
                }
            }
            Path[] finalPathsArray = new Path[parts.size()];
            for (int idx = 0; idx < parts.size(); idx++) {
                finalPathsArray[idx] = (Path) parts.get(idx);
            }
            return path(finalPathsArray);
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
                if (isBlockedLine(a[0], a[1], b[0], b[1])) continue;
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
            }
        }
        if (dist[1] == Double.MAX_VALUE) return null;
        List<double[]> pathPoints = new ArrayList<>();
        int curr = prev[1];
        while (curr != 0 && curr != -1) {
            pathPoints.add(nodes.get(curr));
            curr = prev[curr];
        }
        Collections.reverse(pathPoints);
        return pathPoints;
    }

    private boolean isBlockedLine(double x1, double y1, double x2, double y2) {
        for (double[] zone : KEEP_OUT_ZONES) {
            if (!active(zone)) continue;
            double minX = Math.min(zone[0], zone[2]) - ROBOT_MARGIN;
            double maxX = Math.max(zone[0], zone[2]) + ROBOT_MARGIN;
            double minY = Math.min(zone[1], zone[3]) - ROBOT_MARGIN;
            double maxY = Math.max(zone[1], zone[3]) + ROBOT_MARGIN;
            if (segmentHitsRect(x1, y1, x2, y2, minX, minY, maxX, maxY)) return true;
        }
        return false;
    }

    private boolean segmentsClear(List<double[][]> segs) {
        for (double[][] sg : segs) {
            if (sg.length == 2) {
                if (isBlockedLine(sg[0][0], sg[0][1], sg[1][0], sg[1][1])) return false;
            } else if (sg.length == 3) {
                for (int i = 0; i <= CURVE_SAMPLES; i++) {
                    double t = (double) i / CURVE_SAMPLES;
                    double x = (1 - t) * (1 - t) * sg[0][0] + 2 * (1 - t) * t * sg[1][0] + t * t * sg[2][0];
                    double y = (1 - t) * (1 - t) * sg[0][1] + 2 * (1 - t) * t * sg[1][1] + t * t * sg[2][1];
                    if (insideAnyZone(x, y)) return false;
                }
            }
        }
        return true;
    }

    private boolean insideAnyZone(double x, double y) {
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

    private String findNearestKnownFilledTile(double rx, double ry) {
        String nearestKey = null;
        double minDistance = Double.MAX_VALUE;
        for (Map.Entry<String, TrackedTile> entry : spatialTileMap.entrySet()) {
            TrackedTile tile = entry.getValue();
            if (!tile.hasBall) continue;
            double tileGlobalX = (tile.tileX * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);
            double tileGlobalY = (tile.tileY * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);
            double dist = Math.hypot(tileGlobalX - rx, tileGlobalY - ry);
            if (dist < minDistance) {
                minDistance = dist;
                nearestKey = entry.getKey();
            }
        }
        return nearestKey;
    }

    private void drive(double forward, double lateral, double turn) {
        double fl = forward + lateral + turn;
        double fr = forward - lateral - turn;
        double bl = forward - lateral + turn;
        double br = forward + lateral - turn;
        double maxPow = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (maxPow > 1.0) { fl /= maxPow; fr /= maxPow; bl /= maxPow; br /= maxPow; }
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void lockCameraSettings(VisionPortal portal) {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposure.setExposure(15, TimeUnit.MILLISECONDS);
            sleep(20);
            portal.getCameraControl(GainControl.class).setGain(25);
            sleep(20);
        } catch (Exception ignored) {}
    }
}