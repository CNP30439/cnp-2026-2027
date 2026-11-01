package org.firstinspires.ftc.teamcode.blobcv;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


// ── PEDRO PATHING LIBRARIES INTEGRATION ──────────────────────────────────
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths;
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


@TeleOp(name = "BlobChaseTestV2")
public class BlobChaseTestV2 extends LinearOpMode {


    private static final int IMAGE_WIDTH  = 320;
    private static final int IMAGE_HEIGHT = 240;
    private static final double TARGET_X = IMAGE_WIDTH / 2.0;
    private static final double TURN_DIRECTION = 1.0;


    private static final long EXPOSURE_MS = 5;
    private static final int  GAIN        = 30;


    private static final ColorRange TARGET_COLOR = new ColorRange(
            ColorSpace.HSV,
            new Scalar(15,  80,  50),
            new Scalar(35, 255, 255)
    );


    private static final int ERODE_SIZE  = 10;
    private static final int DILATE_SIZE = 11;


    private static final double MIN_BLOB_AREA   = 0;
    private static final double MIN_CIRCULARITY = 0;


    private static final double TARGET_AREA = 3000;


    private static final double TURN_GAIN    = 0.005;
    private static final double FORWARD_GAIN = 0.8;
    private static final double MAX_TURN     = 0.5;
    private static final double MAX_FORWARD  = 0.80;


    private static final double CENTER_TOLERANCE_PX = 12;


    private static final double FINAL_APPROACH_POWER         = 0.5;
    private static final double FINAL_APPROACH_TIME_S        = 0.7;
    private static final double FINAL_APPROACH_TRIGGER_SPEED = 0.13;


    // ── GAME STYLE TILES CONFIGURATION ──────────────────────────────────────
    private static final double TILE_SIZE_INCHES = 0.5;


    // ── BIOBUZZ FIELD STRUCTURE DODGING PARAMETERS ──────────────────────────
    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91},   // BioBuzz Obstacle Structure 1 {minX, minY, maxX, maxY}
            {91, 50, 96, 91}    // BioBuzz Obstacle Structure 2 {minX, minY, maxX, maxY}
    };


    private static final double ROBOT_MARGIN  = 7.0;
    private static final double CORNER_RADIUS = 10.0;
    private static final int    CURVE_SAMPLES = 20;
    private static final double PAD_STEP      = 1.5;
    private static final int    PAD_TRIES     = 10;
    private static final double FIELD_MIN     = 0;
    private static final double FIELD_MAX     = 144;


    private final PoseFactory pFactory = PoseFactory.degrees();


    // ── PEDRO PATHING INSTANCE REFERENCE ───────────────────────────────────
    private Follower follower;


    private final Map<String, TrackedTile> spatialTileMap = new HashMap<>();
    private String currentLockedTargetTileKey = null;
    private boolean lastBallChaseBtn = false;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private static final double DRIVE_DEADZONE = 0.05;


    private enum ChaseState { IDLE, CHASING, FINAL_APPROACH, DONE, PEDRO_DRIVING }
    private ChaseState chaseFsmState = ChaseState.IDLE;


    private final com.qualcomm.robotcore.util.ElapsedTime finalApproachTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();


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


    // ── CAMERA STREAM HIGHLIGHTER ───────────────────────────────────────────
    // Draws a translucent yellow fill on every detected ball, and a thick
    // green ring on the largest one (the one the chase logic will target).
    private static class BallHighlighter implements VisionProcessor {
        private final ColorBlobLocatorProcessor source;
        private final Paint fillPaint = new Paint();
        private final Paint ringPaint = new Paint();


        BallHighlighter(ColorBlobLocatorProcessor source) {
            this.source = source;


            fillPaint.setStyle(Paint.Style.FILL);
            fillPaint.setColor(Color.argb(110, 255, 230, 0));   // translucent yellow


            ringPaint.setStyle(Paint.Style.STROKE);
            ringPaint.setStrokeWidth(6);
            ringPaint.setColor(Color.rgb(0, 255, 0));           // green ring on target
        }


        @Override
        public void init(int width, int height, CameraCalibration calibration) {}


        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }


        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            List<ColorBlobLocatorProcessor.Blob> blobs = source.getBlobs();


            ColorBlobLocatorProcessor.Blob best = null;
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                if (b.getContourArea() < MIN_BLOB_AREA) continue;
                if (best == null || b.getContourArea() > best.getContourArea()) best = b;
            }


            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                if (b.getContourArea() < MIN_BLOB_AREA) continue;
                Circle c = b.getCircle();
                float cx = (float) c.getX() * scaleBmpPxToCanvasPx;
                float cy = (float) c.getY() * scaleBmpPxToCanvasPx;
                float r  = (float) c.getRadius() * scaleBmpPxToCanvasPx;


                canvas.drawCircle(cx, cy, r, fillPaint);
                if (b == best) canvas.drawCircle(cx, cy, r + 8, ringPaint);
            }
        }
    }


    @Override
    public void runOpMode() {
        Scheduler.reset();


        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(109.5, 132, Math.toRadians(90)));
        follower.update();


        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(TARGET_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(ERODE_SIZE)
                .setDilateSize(DILATE_SIZE)
                .build();


        // Registered AFTER the color locator so its drawing goes on top
        BallHighlighter highlighter = new BallHighlighter(colorLocator);


        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .addProcessor(colorLocator)
                .addProcessor(highlighter)
                .build();


        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        lockCameraSettings(portal);


        telemetry.addLine("Pedro Localizer Linked. Press A to select CLOSEST ball and avoid obstacles.");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();


            Pose robotPose = follower.pose();
            double robotGlobalX = robotPose.x();
            double robotGlobalY = robotPose.y();
            double robotGlobalHeadingRad = robotPose.heading();


            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Blob best = null;


            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                double area = b.getContourArea();
                if (area < MIN_BLOB_AREA) continue;
                double circ = circularity(b.getContour(), area);
                if (circ < MIN_CIRCULARITY) continue;
                if (best == null || area > best.getContourArea()) {
                    best = b;
                }
            }


            // ── SPATIAL GRID MAPPING PASS ─────────────────────────────────────────
            if (best != null) {
                double rawArea = best.getContourArea();
                double centerImgX = best.getBoxFit().center.x;


                double estimatedDistanceInches = Math.sqrt(TARGET_AREA / Math.max(1, rawArea)) * 12.0;
                double angleOffsetRad = ((centerImgX - TARGET_X) / IMAGE_WIDTH) * Math.toRadians(60.0);
                double absoluteAngleRad = robotGlobalHeadingRad + angleOffsetRad;


                double ballGlobalX = robotGlobalX + (estimatedDistanceInches * Math.cos(absoluteAngleRad));
                double ballGlobalY = robotGlobalY + (estimatedDistanceInches * Math.sin(absoluteAngleRad));


                int tileX = (int) Math.floor(ballGlobalX / TILE_SIZE_INCHES);
                int tileY = (int) Math.floor(ballGlobalY / TILE_SIZE_INCHES);
                String discoveredTileKey = tileX + "," + tileY;


                if (currentLockedTargetTileKey == null || currentLockedTargetTileKey.equals(discoveredTileKey)) {
                    if (!spatialTileMap.containsKey(discoveredTileKey)) {
                        spatialTileMap.put(discoveredTileKey, new TrackedTile(tileX, tileY));
                    } else {
                        spatialTileMap.get(discoveredTileKey).lastSeenTimestamp = System.currentTimeMillis();
                        spatialTileMap.get(discoveredTileKey).hasBall = true;
                    }
                }
            }


            // ── TRIGGER AUTO PATH TO CLOSEST BALL ────────────────────────────────
            boolean ballChaseBtn = gamepad1.a;
            boolean ballChaseRising = ballChaseBtn && !lastBallChaseBtn;
            lastBallChaseBtn = ballChaseBtn;


            if (ballChaseRising && (chaseFsmState != ChaseState.PEDRO_DRIVING)) {
                String closestTargetKey = findNearestKnownFilledTile(robotGlobalX, robotGlobalY);
                if (closestTargetKey != null) {
                    currentLockedTargetTileKey = closestTargetKey;
                    String[] coords = closestTargetKey.split(",");
                    double targetFieldX = (Integer.parseInt(coords[0]) * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);
                    double targetFieldY = (Integer.parseInt(coords[1]) * TILE_SIZE_INCHES) + (TILE_SIZE_INCHES / 2.0);


                    Pose targetPose = new Pose(targetFieldX, targetFieldY, robotGlobalHeadingRad);
                    scheduleRoute(robotPose, targetPose);
                    chaseFsmState = ChaseState.PEDRO_DRIVING;
                }
            }


            // ── DRIVETRAIN STATE MACHINE EVALUATION ──────────────────────────────
            double forward = 0, lateral = 0, turn = 0;
            String modeStatusText = "MANUAL CONTROL";


            if (chaseFsmState == ChaseState.PEDRO_DRIVING) {
                modeStatusText = "AUTONOMOUS CHASE — SEEKING LOCKED BALL";


                if (!follower.isBusy()) {
                    Scheduler.reset();
                    if (currentLockedTargetTileKey != null) {
                        if (spatialTileMap.containsKey(currentLockedTargetTileKey)) {
                            spatialTileMap.get(currentLockedTargetTileKey).hasBall = false;
                        }
                        currentLockedTargetTileKey = null;
                    }
                    chaseFsmState = ChaseState.IDLE;
                }
            } else if (gamepad1.right_bumper) {
                lateral = 0;


                if (chaseFsmState == ChaseState.DONE) {
                    if (currentLockedTargetTileKey != null) {
                        if (spatialTileMap.containsKey(currentLockedTargetTileKey)) {
                            spatialTileMap.get(currentLockedTargetTileKey).hasBall = false;
                        }
                        currentLockedTargetTileKey = null;
                    }
                    forward = 0; turn = 0;
                    modeStatusText = "DONE — Target Consumed";
                    if (!gamepad1.right_bumper) chaseFsmState = ChaseState.IDLE;


                } else if (chaseFsmState == ChaseState.FINAL_APPROACH) {
                    if (finalApproachTimer.seconds() >= FINAL_APPROACH_TIME_S) {
                        forward = 0; turn = 0;
                        chaseFsmState = ChaseState.DONE;
                    } else {
                        forward = FINAL_APPROACH_POWER; turn = 0;
                        modeStatusText = "FINAL APPROACH RUN";
                    }


                } else {
                    if (currentLockedTargetTileKey == null && best != null) {
                        double rawArea = best.getContourArea();
                        double estimatedDistanceInches = Math.sqrt(TARGET_AREA / Math.max(1, rawArea)) * 12.0;
                        double angleOffsetRad = ((best.getBoxFit().center.x - TARGET_X) / IMAGE_WIDTH) * Math.toRadians(60.0);
                        double absoluteAngleRad = robotGlobalHeadingRad + angleOffsetRad;


                        int tX = (int) Math.floor((robotGlobalX + estimatedDistanceInches * Math.cos(absoluteAngleRad)) / TILE_SIZE_INCHES);
                        int tY = (int) Math.floor((robotGlobalY + estimatedDistanceInches * Math.sin(absoluteAngleRad)) / TILE_SIZE_INCHES);
                        currentLockedTargetTileKey = tX + "," + tY;
                    }


                    if (best == null) {
                        forward = 0; turn = 0;
                        chaseFsmState = ChaseState.IDLE;
                        modeStatusText = "BLIND ENGINE — NO ACTIVE TARGETS";
                    } else {
                        double xError = best.getBoxFit().center.x - TARGET_X;
                        turn = clamp(xError * TURN_GAIN * TURN_DIRECTION, -MAX_TURN, MAX_TURN);
                        if (Math.abs(xError) < CENTER_TOLERANCE_PX) turn = 0;


                        double sizeError = (TARGET_AREA - best.getContourArea()) / TARGET_AREA;
                        forward = clamp(sizeError * FORWARD_GAIN, 0, MAX_FORWARD);
                        forward *= (1.0 - Math.min(1.0, Math.abs(xError) / (IMAGE_WIDTH / 2.0)));


                        if (forward < FINAL_APPROACH_TRIGGER_SPEED && Math.abs(xError) < CENTER_TOLERANCE_PX) {
                            chaseFsmState = ChaseState.FINAL_APPROACH;
                            finalApproachTimer.reset();
                            forward = FINAL_APPROACH_POWER; turn = 0;
                        } else {
                            chaseFsmState = ChaseState.CHASING;
                            modeStatusText = "PROPORTIONAL EYE ALIGNMENT RUN";
                        }
                    }
                }


                drive(forward, lateral, turn);


            } else {
                forward = -gamepad1.left_stick_y;
                lateral =  gamepad1.left_stick_x;
                turn    =  gamepad1.right_stick_x;


                if (Math.abs(forward) < DRIVE_DEADZONE) forward = 0;
                if (Math.abs(lateral) < DRIVE_DEADZONE) lateral = 0;
                if (Math.abs(turn)    < DRIVE_DEADZONE) turn    = 0;


                drive(forward, lateral, turn);
                chaseFsmState = ChaseState.IDLE;
                currentLockedTargetTileKey = null;
            }


            telemetry.addData("State Status",  modeStatusText);
            telemetry.addData("Robot Pose",    "X: %.1f, Y: %.1f", robotGlobalX, robotGlobalY);
            telemetry.addData("Target Locked", currentLockedTargetTileKey == null ? "Open Field" : currentLockedTargetTileKey);
            telemetry.addData("Cached Targets", spatialTileMap.size());
            telemetry.update();
        }


        drive(0, 0, 0);
        portal.close();
    }


    // ── OBSTACLE AVOIDANCE AND ROAD ROUTING MATHEMATICS ──────────────────────
    private void scheduleRoute(Pose start, Pose target) {
        if (isBlocked(start.x(), start.y(), target.x(), target.y())) {
            Path curvedRoute = buildCurvedPath(start, target);
            if (curvedRoute != null) {
                schedule(follow(follower, curvedRoute));
                return;
            }
        }
        schedule(follow(follower, line(start, target).linear(start, target)));
    }


    @SuppressWarnings({"rawtypes", "unchecked"})
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


            // Explicitly convert the list to an array to resolve the IDE compilation error
            Path[] finalPathsArray = (Path[]) parts.toArray(new Path[0]);
            return com.pedropathing.api.Paths.path(finalPathsArray);
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


            double[] entry = {c[0] - (c[0] - a[0]) / lenIn * rIn,   c[1] - (c[1] - a[1]) / lenIn * rIn};
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


    private static double circularity(MatOfPoint contour, double area) {
        MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
        double perimeter = Imgproc.arcLength(c2f, true);
        c2f.release();
        if (perimeter <= 0) return 0;
        return 4 * Math.PI * area / (perimeter * perimeter);
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
            exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            sleep(20);
            portal.getCameraControl(GainControl.class).setGain(GAIN);
            sleep(20);
        } catch (Exception ignored) {}
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}