///* ============================================================= *
// *                 Turtle Tracer — Auto-Generated                *
// *                                                               *
// *  Version: 2.6.1.                                              *
// *  Copyright (c) 2026 Matthew Allen                             *
// *                                                               *
// *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
// *  Changes will be overwritten when regenerated.                *
// *                                                               *
// *  NOTE: the turret / shooter / intake / routing code below was *
// *  added by hand (ported from TeleMainTurret). Regenerating the *
// *  paths from Turtle Tracer will overwrite it.                  *
// * ============================================================= */
//
//package org.firstinspires.ftc.teamcode.Auto;
//
//import static com.pedropathing.api.Paths.curve;
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.api.Paths.path;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import static com.pedropathing.ivy.pedro.PedroCommands.hold;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Scheduler;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.List;
//
//@Autonomous(name = "2 tip auto", group = "Autonomous")
//public class twotipauto extends OpMode {
//
//    // ── Field data ───────────────────────────────────────────────────────────
//    private enum Alliance { RED, BLUE }
//
//    // The drawn paths are for RED. Changing this to BLUE only changes the goal used for aiming,
//    // it does NOT mirror the paths.
//    private final Alliance alliance = Alliance.RED;
//
//    public static final double[][] RED_GOALS  = { {58, 52}, {58, 90} };
//    public static final double[][] BLUE_GOALS = { {84, 51}, {84, 90} };
//
//    // 0 = the goal used from the bottom shooting zone, 1 = the goal used from the top zone.
//    private int presetIndex = 0;
//
//    // ── Drive / state machine ────────────────────────────────────────────────
//    public Follower follower; // Pathing follower instance
//    private final PoseFactory p = PoseFactory.degrees();
//    private int pathState; // Current autonomous path state (state machine)
//    private ElapsedTime pathTimer; // Timer for path state machine
//    private Paths paths; // Paths defined in the Paths class
//
//    // Shooter spin-up wait before the first path starts.
//    private static final double SPINUP_MS = 3000;
//
//    // How long each shoot stop lasts (includes the kick delay).
//    private static final double SHOOT_MS = 2500;
//
//    // How long the robot stays at the end of line2 (intake keeps running).
//    private static final double LINE2_HOLD_MS = 1500;
//
//    // Set by the state machine every loop.
//    private boolean shooting = false; // feed balls into the shooter
//    private boolean intaking = false; // run the intake while driving
//
//    // ── Shooting-zone bounds (used by computeShootPose) ──────────────────────
//    private static final double PRESET0_MAX_Y = 25;
//    private static final double PRESET1_MIN_Y = 116;
//    private static final double RED_X_MIN = 25, RED_X_MAX = 85;
//    private static final double BLUE_X_MIN = 50, BLUE_X_MAX = 119;
//    private static final double ZONE_MARGIN = 1.0;
//    private static final double WALL_MARGIN = 8.0;
//    private static final double SHOOT_DISTANCE = 41.0;
//    private static final double SHOOTER_OFFSET_IN = 0.0;
//
//    // ── Structure dodging ────────────────────────────────────────────────────
//    public static final double[][] KEEP_OUT_ZONES = {
//            {45, 50, 49, 91}, // STRUCTURE 1
//            {91, 50, 96, 91}  // STRUCTURE 2
//    };
//    private static final double ROBOT_MARGIN = 7.0;
//    private static final double CORNER_RADIUS = 10.0;
//    private static final int CURVE_SAMPLES = 20;
//    private static final double PAD_STEP = 1.5;
//    private static final int PAD_TRIES = 10;
//    private static final double FIELD_MIN = 0;
//    private static final double FIELD_MAX = 144;
//    private int routeCorners = 0;
//
//    // ── Turret tuning ────────────────────────────────────────────────────────
//    private static final String TURRET_MOTOR = "turret";
//
//    private static final double TICKS_PER_MOTOR_REV      = 4000.0;
//    private static final double MOTOR_REV_PER_TURRET_REV = 82.0 / 25.0;
//    private static final double ENCODER_SIGN = 1.0;
//    private static final double POWER_SIGN = 1.0;
//    private static final double TICKS_PER_RAD =
//            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV / (2 * Math.PI);
//
//    private static final boolean TURRET_REVERSED = false;
//    private static final double TURRET_MIN_RAD = Math.toRadians(-95);
//    private static final double TURRET_MAX_RAD = Math.toRadians(275);
//    private static final double TURRET_START_RAD = Math.toRadians(0.0);
//    private static final double TURRET_ZERO_OFFSET_RAD = Math.PI;
//
//    private static final double PIVOT_X_IN = 0.0;
//    private static final double PIVOT_Y_IN = 0.0;
//
//    private static final double kP = 5.0;
//    private static final double kD = 0.12;
//    private static final double kV = 0.17;
//    private static final double kS = 0.04;
//    private static final double KS_RAMP_RAD = Math.toRadians(1.0);
//    private static final double MAX_POWER = 0.96;
//    private static final double SWING_POWER = 0.96;
//    private static final double SWING_ERR_DEG = 25;
//    private static final double TOLERANCE_RAD = Math.toRadians(0.22);
//    private static final double SPIN_FILTER = 0.08;
//    private static final double SHOOT_READY_DEG = 2.0;
//    private static final double SHOOT_READY_RAD = Math.toRadians(SHOOT_READY_DEG);
//
//    private static final double AIM_RIGHT_SIDE_RAD = Math.toRadians(-4.0);
//    private static final double AIM_LEFT_SIDE_RAD  = Math.toRadians(-2.0);
//    private static final double AIM_LEFT_DEADBAND_IN = 25.0;
//
//    // ── Turret state ─────────────────────────────────────────────────────────
//    private DcMotorEx turret;
//    private final ElapsedTime dtTimer = new ElapsedTime();
//    private double lastWanted = 0;
//    private double lastTarget = 0;
//    private double spinRate = 0;
//    private double turretErr = 0;
//    private double turretPower = 0;
//    private boolean turretReady = false;
//
//    // ── Shooter ──────────────────────────────────────────────────────────────
//    private DcMotorEx leftShooter;
//    private static final double SHOOTER_P = 71.5;
//    private static final double SHOOTER_F_LEFT = 12.84;
//    // Ticks/sec the shooter runs at during auto. Tune this for your shot.
//    private static final double AUTO_SHOOTER_VELOCITY = 200;
//
//    // ── Intake ───────────────────────────────────────────────────────────────
//    private DcMotorEx leftIntake, rightIntake;
//    private static final double INTAKE_POWER = 1.0;
//
//    // ── Flower ───────────────────────────────────────────────────────────────
//    private Servo flowerA, flowerB;
//    private static final String FLOWER_A_NAME = "linkageright"; // FORWARD
//    private static final String FLOWER_B_NAME = "linkageleft";  // REVERSED
//    private static final double FLOWER_DOWN = 0.153;
//    private static final double FLOWER_UP = 0.7;
//
//    // ── Blocker / kicker ─────────────────────────────────────────────────────
//    private Servo blocker;
//    private static final double BLOCKER_CLOSED = 1.0;
//    private static final double BLOCKER_OPEN = 0.6;
//
//    private Servo kicker;
//    private static final double KICKER_BASE = 0.5;
//    private static final double KICKER_KICK = 0.8;
//    private static final long KICK_DELAY_MS = 500;
//    private final ElapsedTime kickTimer = new ElapsedTime();
//    private boolean lastShooting = false;
//    private boolean kickReady = false;
//
//    // ── Prism ────────────────────────────────────────────────────────────────
//    private GoBildaPrismDriver prism;
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // Helpers
//    // ═════════════════════════════════════════════════════════════════════════
//    private double[] goal() {
//        return (alliance == Alliance.RED ? RED_GOALS : BLUE_GOALS)[presetIndex];
//    }
//
//    /** Red: robot y < goal y = right side of goal. Blue is mirrored. */
//    private boolean onRightSideOfGoal(Pose pose) {
//        return (alliance == Alliance.RED) == (pose.y() < goal()[1]);
//    }
//
//    private static double wrap(double a) {
//        return Math.atan2(Math.sin(a), Math.cos(a));
//    }
//
//    private static double angleDiff(double a, double b) {
//        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
//    }
//
//    private static double clamp(double v, double lo, double hi) {
//        return Math.max(lo, Math.min(hi, v));
//    }
//
//    private double turretAngleRad() {
//        return TURRET_START_RAD + ENCODER_SIGN * turret.getCurrentPosition() / TICKS_PER_RAD;
//    }
//
//    private double turretSpeedRadPerSec() {
//        return ENCODER_SIGN * turret.getVelocity() / TICKS_PER_RAD;
//    }
//
//    /** Turret angle (rad, turret frame) that points the shooter at the goal right now. */
//    private double wantedTurretAngle(Pose pose) {
//        double h = pose.heading();
//        double px = pose.x() + PIVOT_X_IN * Math.cos(h) - PIVOT_Y_IN * Math.sin(h);
//        double py = pose.y() + PIVOT_X_IN * Math.sin(h) + PIVOT_Y_IN * Math.cos(h);
//        double[] g = goal();
//        double fieldAngle = Math.atan2(g[1] - py, g[0] - px);
//        return wrap(fieldAngle - h - TURRET_ZERO_OFFSET_RAD);
//    }
//
//    /** Picks which equivalent angle (+-360 deg) the turret goes to, staying inside its range. */
//    private double fitToRange(double angle, double prevTarget) {
//        double w = wrap(angle);
//        double twoPi = 2 * Math.PI;
//        double cont = w + Math.round((prevTarget - w) / twoPi) * twoPi;
//        if (cont >= TURRET_MIN_RAD && cont <= TURRET_MAX_RAD) return cont;
//
//        double best = Double.NaN;
//        for (int k = -1; k <= 1; k++) {
//            double a = w + k * twoPi;
//            if (a < TURRET_MIN_RAD || a > TURRET_MAX_RAD) continue;
//            if (Double.isNaN(best) || Math.abs(a - prevTarget) < Math.abs(best - prevTarget)) best = a;
//        }
//        if (!Double.isNaN(best)) return best;
//        return Math.max(TURRET_MIN_RAD, Math.min(TURRET_MAX_RAD, cont));
//    }
//
//    private void setFlowerPosition(double position) {
//        flowerA.setPosition(position);
//        flowerB.setPosition(position);
//    }
//
//    /** Heading (radians) that points the back of the robot (turret at zero) at the goal from (x, y). */
//    private double headingToGoal(double x, double y) {
//        double[] g = goal();
//        return Math.atan2(g[1] - y, g[0] - x) + TURRET_ZERO_OFFSET_RAD;
//    }
//
//    /** Nearest legal shooting pose in the zone, heading lined up on the goal. */
//    private Pose computeShootPose() {
//        Pose cur = follower.pose();
//        double xMin = alliance == Alliance.RED ? RED_X_MIN : BLUE_X_MIN;
//        double xMax = alliance == Alliance.RED ? RED_X_MAX : BLUE_X_MAX;
//        double x = clamp(cur.x(), xMin + ZONE_MARGIN, xMax - ZONE_MARGIN);
//        double yLo = (presetIndex == 0) ? FIELD_MIN + WALL_MARGIN : PRESET1_MIN_Y + ZONE_MARGIN;
//        double yHi = (presetIndex == 0) ? PRESET0_MAX_Y - ZONE_MARGIN : FIELD_MAX - WALL_MARGIN;
//        double y = clamp(cur.y(), yLo, yHi);
//
//        double[] g = goal();
//        double dx = x - g[0];
//        double centerDist = SHOOT_DISTANCE + SHOOTER_OFFSET_IN;
//        if (centerDist > Math.abs(dx)) {
//            double dy = Math.sqrt(centerDist * centerDist - dx * dx);
//            double idealY = (presetIndex == 0) ? g[1] - dy : g[1] + dy;
//            y = clamp(idealY, yLo, yHi);
//        }
//        return p.of(x, y, Math.toDegrees(headingToGoal(x, y)));
//    }
//
//    /** Aim error (radians) from the robot's current position: + = shot too far left. */
//    private double aimErrorRad() {
//        Pose now = follower.pose();
//        return angleDiff(now.heading(), headingToGoal(now.x(), now.y()));
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // Routing around the structures
//    // ═════════════════════════════════════════════════════════════════════════
//
//    /** Straight line from the robot's current pose to the given target. */
//    private Path pathTo(Pose target) {
//        Pose currentPose = follower.pose();
//        return line(currentPose, target).linear(currentPose, target);
//    }
//
//    /** Schedules a drive to the target: straight if clear, else one rounded path around the structures. */
//    private void scheduleRoute(Pose target) {
//        Pose start = follower.pose();
//        routeCorners = 0;
//        if (isBlocked(start.x(), start.y(), target.x(), target.y())) {
//            Path route = buildCurvedPath(start, target);
//            if (route != null) {
//                schedule(follow(follower, route));
//                return;
//            }
//        }
//        schedule(follow(follower, pathTo(target)));
//    }
//
//    private Path buildCurvedPath(Pose start, Pose target) {
//        double pad = ROBOT_MARGIN + CORNER_RADIUS * 0.5;
//        for (int attempt = 0; attempt < PAD_TRIES; attempt++, pad += PAD_STEP) {
//            List<double[]> corners = findCorners(start, target, pad);
//            if (corners == null || corners.isEmpty()) return null;
//
//            List<double[]> pts = new ArrayList<>();
//            pts.add(new double[]{start.x(), start.y()});
//            pts.addAll(corners);
//            pts.add(new double[]{target.x(), target.y()});
//
//            List<double[][]> segs = roundedSegments(pts);
//            if (!segmentsClear(segs, start, target)) continue;
//
//            double h0 = start.heading();
//            double delta = Math.atan2(Math.sin(target.heading() - h0), Math.cos(target.heading() - h0));
//            double total = 0;
//            for (double[][] sg : segs) total += controlLength(sg);
//
//            List<Path> parts = new ArrayList<>();
//            double run = 0;
//            for (double[][] sg : segs) {
//                double len = controlLength(sg);
//                double hA = h0 + delta * (run / total);
//                double hB = h0 + delta * ((run + len) / total);
//                run += len;
//
//                Pose a = p.of(sg[0][0], sg[0][1], Math.toDegrees(hA));
//                Pose b = p.of(sg[sg.length - 1][0], sg[sg.length - 1][1], Math.toDegrees(hB));
//                if (sg.length == 2) {
//                    parts.add(line(a, b).linear(a, b));
//                } else {
//                    Pose m = p.of(sg[1][0], sg[1][1], Math.toDegrees(hA));
//                    parts.add(curve(a, m, b).linear(a, b));
//                }
//            }
//            routeCorners = corners.size();
//            return path(parts.toArray(new Path[0]));
//        }
//        return null;
//    }
//
//    private List<double[][]> roundedSegments(List<double[]> pts) {
//        List<double[][]> out = new ArrayList<>();
//        double[] cur = pts.get(0);
//        for (int k = 1; k < pts.size() - 1; k++) {
//            double[] a = pts.get(k - 1), c = pts.get(k), b = pts.get(k + 1);
//            double lenIn = Math.hypot(c[0] - a[0], c[1] - a[1]);
//            double lenOut = Math.hypot(b[0] - c[0], b[1] - c[1]);
//            double rIn = Math.min(CORNER_RADIUS, lenIn / 2);
//            double rOut = Math.min(CORNER_RADIUS, lenOut / 2);
//            double[] entry = {c[0] - (c[0] - a[0]) / lenIn * rIn, c[1] - (c[1] - a[1]) / lenIn * rIn};
//            double[] exit = {c[0] + (b[0] - c[0]) / lenOut * rOut, c[1] + (b[1] - c[1]) / lenOut * rOut};
//            if (Math.hypot(entry[0] - cur[0], entry[1] - cur[1]) > 0.01) {
//                out.add(new double[][]{cur, entry});
//            }
//            out.add(new double[][]{entry, c, exit});
//            cur = exit;
//        }
//        out.add(new double[][]{cur, pts.get(pts.size() - 1)});
//        return out;
//    }
//
//    private static double controlLength(double[][] sg) {
//        double len = 0;
//        for (int i = 1; i < sg.length; i++) {
//            len += Math.hypot(sg[i][0] - sg[i - 1][0], sg[i][1] - sg[i - 1][1]);
//        }
//        return len;
//    }
//
//    private List<double[]> findCorners(Pose start, Pose target, double pad) {
//        List<double[]> nodes = new ArrayList<>();
//        nodes.add(new double[]{start.x(), start.y()});
//        nodes.add(new double[]{target.x(), target.y()});
//        for (double[] z : KEEP_OUT_ZONES) {
//            if (!active(z)) continue;
//            double[][] corners = {
//                    {z[0] - pad, z[1] - pad},
//                    {z[2] + pad, z[1] - pad},
//                    {z[2] + pad, z[3] + pad},
//                    {z[0] - pad, z[3] + pad}
//            };
//            for (double[] c : corners) {
//                boolean onField = c[0] >= FIELD_MIN && c[0] <= FIELD_MAX
//                        && c[1] >= FIELD_MIN && c[1] <= FIELD_MAX;
//                if (onField && !insideAnyZone(c[0], c[1])) nodes.add(c);
//            }
//        }
//
//        int n = nodes.size();
//        double[] dist = new double[n];
//        int[] prev = new int[n];
//        boolean[] done = new boolean[n];
//        java.util.Arrays.fill(dist, Double.MAX_VALUE);
//        java.util.Arrays.fill(prev, -1);
//        dist[0] = 0;
//        for (int iter = 0; iter < n; iter++) {
//            int u = -1;
//            for (int i = 0; i < n; i++) {
//                if (!done[i] && dist[i] < Double.MAX_VALUE && (u == -1 || dist[i] < dist[u])) u = i;
//            }
//            if (u == -1) break;
//            done[u] = true;
//            for (int v = 0; v < n; v++) {
//                if (done[v]) continue;
//                double[] a = nodes.get(u), b = nodes.get(v);
//                if (isBlocked(a[0], a[1], b[0], b[1])) continue;
//                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]);
//                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
//            }
//        }
//        if (dist[1] == Double.MAX_VALUE) return null;
//
//        List<Integer> chain = new ArrayList<>();
//        for (int at = 1; at != -1; at = prev[at]) chain.add(at);
//        Collections.reverse(chain);
//        List<double[]> out = new ArrayList<>();
//        for (int i = 1; i < chain.size() - 1; i++) out.add(nodes.get(chain.get(i)));
//        return out;
//    }
//
//    private boolean segmentsClear(List<double[][]> segs, Pose start, Pose target) {
//        final double eps = 0.01;
//        for (double[][] sg : segs) {
//            for (int i = 0; i <= CURVE_SAMPLES; i++) {
//                double[] pt = bezierPoint(sg, (double) i / CURVE_SAMPLES);
//                for (double[] z : KEEP_OUT_ZONES) {
//                    if (!active(z)) continue;
//                    double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
//                    double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
//                    if (inside(start.x(), start.y(), x0, y0, x1, y1)
//                            || inside(target.x(), target.y(), x0, y0, x1, y1)) continue;
//                    if (inside(pt[0], pt[1], x0, y0, x1, y1)) return false;
//                }
//            }
//        }
//        return true;
//    }
//
//    private static double[] bezierPoint(double[][] ctrl, double t) {
//        int n = ctrl.length;
//        double[] x = new double[n], y = new double[n];
//        for (int i = 0; i < n; i++) { x[i] = ctrl[i][0]; y[i] = ctrl[i][1]; }
//        for (int k = n - 1; k > 0; k--) {
//            for (int i = 0; i < k; i++) {
//                x[i] = (1 - t) * x[i] + t * x[i + 1];
//                y[i] = (1 - t) * y[i] + t * y[i + 1];
//            }
//        }
//        return new double[]{x[0], y[0]};
//    }
//
//    private boolean isBlocked(double ax, double ay, double bx, double by) {
//        final double eps = 0.01;
//        for (double[] z : KEEP_OUT_ZONES) {
//            if (!active(z)) continue;
//            double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
//            double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
//            if (inside(ax, ay, x0, y0, x1, y1) || inside(bx, by, x0, y0, x1, y1)) continue;
//            if (segmentHitsRect(ax, ay, bx, by, x0, y0, x1, y1)) return true;
//        }
//        return false;
//    }
//
//    private boolean insideAnyZone(double x, double y) {
//        final double eps = 0.01;
//        for (double[] z : KEEP_OUT_ZONES) {
//            if (!active(z)) continue;
//            if (inside(x, y, z[0] - ROBOT_MARGIN + eps, z[1] - ROBOT_MARGIN + eps,
//                    z[2] + ROBOT_MARGIN - eps, z[3] + ROBOT_MARGIN - eps)) return true;
//        }
//        return false;
//    }
//
//    private static boolean active(double[] z) {
//        return !(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 0);
//    }
//
//    private static boolean inside(double x, double y, double x0, double y0, double x1, double y1) {
//        return x > x0 && x < x1 && y > y0 && y < y1;
//    }
//
//    /** Liang–Barsky segment vs. axis-aligned rectangle test. */
//    private static boolean segmentHitsRect(double ax, double ay, double bx, double by,
//                                           double x0, double y0, double x1, double y1) {
//        double t0 = 0, t1 = 1;
//        double dx = bx - ax, dy = by - ay;
//        double[] pp = {-dx, dx, -dy, dy};
//        double[] qq = {ax - x0, x1 - ax, ay - y0, y1 - ay};
//        for (int i = 0; i < 4; i++) {
//            if (pp[i] == 0) {
//                if (qq[i] < 0) return false;
//            } else {
//                double t = qq[i] / pp[i];
//                if (pp[i] < 0) {
//                    if (t > t1) return false;
//                    if (t > t0) t0 = t;
//                } else {
//                    if (t < t0) return false;
//                    if (t < t1) t1 = t;
//                }
//            }
//        }
//        return true;
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // OpMode
//    // ═════════════════════════════════════════════════════════════════════════
//    @Override
//    public void init() {
//        Scheduler.reset();
//
//        follower = Constants.create(hardwareMap);
//        follower.setPose(p.of(58.364, 9.663, -90.000));
//
//        pathTimer = new ElapsedTime();
//        paths = new Paths(follower); // Build paths
//
//        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
//        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//
//        // Turret must be at its zero (facing back) at INIT: the encoder is zeroed here.
//        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
//        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftShooter = hardwareMap.get(DcMotorEx.class, "ls");
//        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT));
//
//        leftIntake = hardwareMap.get(DcMotorEx.class, "li");
//        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
//        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        flowerA = hardwareMap.get(Servo.class, FLOWER_A_NAME);
//        flowerB = hardwareMap.get(Servo.class, FLOWER_B_NAME);
//        flowerA.setDirection(Servo.Direction.FORWARD);
//        flowerB.setDirection(Servo.Direction.REVERSE);
//        setFlowerPosition(FLOWER_UP);
//
//        blocker = hardwareMap.get(Servo.class, "blocker");
//        blocker.setPosition(BLOCKER_CLOSED);
//
//        kicker = hardwareMap.get(Servo.class, "kicker");
//        kicker.setPosition(KICKER_BASE);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addLine("Turret must be at its zero position NOW (encoder is zeroed in INIT)");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        presetIndex = 0;
//        lastWanted = wantedTurretAngle(follower.pose());
//        lastTarget = turretAngleRad();
//        dtTimer.reset();
//        kickTimer.reset();
//        setPathState(0);
//        leftShooter.setVelocity(AUTO_SHOOTER_VELOCITY); // starts spinning up now, stays on the whole auto
//    }
//
//    @Override
//    public void loop() {
//        follower.update(); // Update follower
//        Scheduler.execute();
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//        updateTurret();
//        updateMechanisms();
//
//        // Log values to Driver Station
//        Pose pose = follower.pose();
//        double[] g = goal();
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("X", pose.x());
//        telemetry.addData("Y", pose.y());
//        telemetry.addData("Heading", pose.heading());
//        telemetry.addData("Goal", "(%.0f, %.0f) preset %d", g[0], g[1], presetIndex);
//        telemetry.addData("Distance to goal", "%.1f in", Math.hypot(g[0] - pose.x(), g[1] - pose.y()));
//        telemetry.addData("Shooter", "target=%.0f actual=%.0f",
//                AUTO_SHOOTER_VELOCITY, leftShooter.getVelocity());
//        telemetry.addData("Shooting", shooting ? (kickReady ? "KICK" : "feeding") : "no");
//        telemetry.addData("Intake", intaking || shooting ? "ON" : "off");
//        telemetry.addData("Turret", "now=%.1f err=%.2f deg %s",
//                Math.toDegrees(turretAngleRad()), Math.toDegrees(turretErr),
//                Math.abs(turretErr) <= TOLERANCE_RAD ? "(ON TARGET)"
//                        : (Math.abs(turretErr) > Math.toRadians(SWING_ERR_DEG) ? "(far off)" : ""));
//        telemetry.addData("Turret power", "%.2f", turretPower);
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        Scheduler.reset();
//        if (turret != null) turret.setPower(0);
//        if (leftShooter != null) leftShooter.setPower(0);
//        if (leftIntake != null) leftIntake.setPower(0);
//        if (rightIntake != null) rightIntake.setPower(0);
//        if (flowerA != null) setFlowerPosition(FLOWER_DOWN);
//        if (blocker != null) blocker.setPosition(BLOCKER_CLOSED);
//        if (kicker != null) kicker.setPosition(KICKER_BASE);
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // Turret (always tracking the goal during auto)
//    // ═════════════════════════════════════════════════════════════════════════
//    private void updateTurret() {
//        Pose pose = follower.pose();
//        double dt = Math.max(dtTimer.seconds(), 1e-3);
//        dtTimer.reset();
//
//        double wanted = wantedTurretAngle(pose);
//        double rawRate = wrap(wanted - lastWanted) / dt;
//        lastWanted = wanted;
//        spinRate += SPIN_FILTER * (rawRate - spinRate);
//
//        boolean rightSide = onRightSideOfGoal(pose);
//        boolean nearLeft = !rightSide && Math.abs(pose.y() - goal()[1]) <= AIM_LEFT_DEADBAND_IN;
//        double sideOffset = (rightSide || nearLeft) ? AIM_RIGHT_SIDE_RAD : AIM_LEFT_SIDE_RAD;
//
//        double current = turretAngleRad();
//        double target = fitToRange(wanted + sideOffset, lastTarget);
//        lastTarget = target;
//        turretErr = target - current;
//
//        double ksRamp = Math.max(-1.0, Math.min(1.0, turretErr / KS_RAMP_RAD));
//        double power = kP * turretErr - kD * turretSpeedRadPerSec() + ksRamp * kS + kV * spinRate;
//        double cap = Math.abs(turretErr) > Math.toRadians(SWING_ERR_DEG) ? SWING_POWER : MAX_POWER;
//        power = Math.max(-cap, Math.min(cap, power));
//        turretPower = power;
//        turret.setPower(POWER_SIGN * power);
//
//        turretReady = Math.abs(turretErr) <= SHOOT_READY_RAD;
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // Intake / blocker / kicker
//    // ═════════════════════════════════════════════════════════════════════════
//    private void updateMechanisms() {
//        if (shooting && !lastShooting) kickTimer.reset();
//        lastShooting = shooting;
//        kickReady = shooting && turretReady && kickTimer.milliseconds() >= KICK_DELAY_MS;
//
//        if (shooting) {
//            leftIntake.setPower(INTAKE_POWER);
//            rightIntake.setPower(INTAKE_POWER);
//            blocker.setPosition(BLOCKER_OPEN);
//            kicker.setPosition(kickReady ? KICKER_KICK : KICKER_BASE);
//        } else {
//            double ip = intaking ? INTAKE_POWER : 0;
//            leftIntake.setPower(ip);
//            rightIntake.setPower(ip);
//            blocker.setPosition(BLOCKER_CLOSED);
//            kicker.setPosition(KICKER_BASE);
//        }
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // Paths (from Turtle Tracer)
//    // ═════════════════════════════════════════════════════════════════════════
//    public static class Paths {
//
//        private static final PoseFactory p = PoseFactory.degrees();
//        public Path DriveToShoot;
//        public Path line2;
//        public Path line3;
//        public Path line4;
//        public Path line5;
//        public Path line6;
//
//        public Paths(Follower follower) {
//            DriveToShoot = line(
//                    p.of(58.364, 9.663, 0.0),
//                    p.of(58.364, 21.728, 0.0)
//            ).linear(Math.toRadians(-90), Math.toRadians(-90));
//
//            line2 = curve(
//                    p.of(58.364, 21.728, 0.0),
//                    p.of(49.000, 9.663, 0.0),
//                    p.of(11.169, 10.000, 0.0)
//            ).linear(Math.toRadians(-90), Math.toRadians(-90));
//
//            line3 = curve(
//                    p.of(11.169, 10.000, 0.0),
//                    p.of(31.504, 50.321, 0.0),
//                    p.of(21.682, 124.175, 0.0),
//                    p.of(58.364, 120.377, 0.0)
//            ).linear(Math.toRadians(-90), Math.toRadians(90));
//
//            line4 = line(
//                    p.of(58.364, 120.377, 0.0),
//                    p.of(49.000, 134.000, 0.0)
//            ).linear(Math.toRadians(90), Math.toRadians(90));
//
//            line5 = line(
//                    p.of(49.000, 134.000, 0.0),
//                    p.of(58.364, 120.377, 0.0)
//            ).linear(Math.toRadians(90), Math.toRadians(90));
//
//            line6 = curve(
//                    p.of(58.364, 120.377, 0.0),
//                    p.of(32.330, 118.316, 0.0),
//                    p.of(11.169, 105.933, 0.0)
//            ).linear(Math.toRadians(90), Math.toRadians(90));
//        }
//    }
//
//    // ═════════════════════════════════════════════════════════════════════════
//    // State machine
//    //   0      spin up the shooter for SPINUP_MS (3 s), robot stays still
//    //   1      path 1 (DriveToShoot): drive to the shooting position
//    //   2      SHOOT
//    //   3-5    path 2 (line2): intake balls from the human player zone, then stay LINE2_HOLD_MS (1.5 s)
//    //   6-7    path 3 (line3): go to the other side to shoot (turret switches to the top goal),
//    //          flower (linkage servos) goes DOWN when it finishes
//    //   8      SHOOT
//    //   9-10   path 4 (line4): pick up balls from the flower
//    //   11-12  path 5 (line5): back to shoot, flower goes UP when it finishes
//    //   13     SHOOT
//    //   14-15  path 6 (line6): park
//    //   16     stop
//    //
//    // The intake runs the whole auto (intaking is set to true at the top of every loop).
//    // To stop it at a step, set intaking = false; inside that case.
//    // shooting resets to false at the top of every loop, so set it in every loop of the case.
//    // For timed shooting, use: if (pathTimer.milliseconds() >= SHOOT_MS) setPathState(next);
//    // To change which goal the turret aims at, set presetIndex = 0 or 1.
//    // ═════════════════════════════════════════════════════════════════════════
//    public int autonomousPathUpdate() {
//        shooting = false;
//        intaking = true; // intake runs the ENTIRE auto
//
//        switch (pathState) {
//            case 0: // wait for the shooter to spin up
//                if (pathTimer.milliseconds() >= SPINUP_MS) {
//                    follower.follow(paths.DriveToShoot);
//                    setPathState(1);
//                }
//                break;
//            case 1: // path 1: driving to the shooting position
//                if (!follower.isBusy()) {
//                    setPathState(2);
//                }
//                break;
//            case 2: // shoot
//                shooting = true;
//                if (pathTimer.milliseconds() >= SHOOT_MS) {
//                    setPathState(3);
//                }
//                break;
//            case 3: // start path 2
//                follower.follow(paths.line2);
//                setPathState(4);
//                break;
//            case 4: // path 2: intaking from the human player zone
//                if (!follower.isBusy()) {
//                    setPathState(5);
//                }
//                break;
//            case 5: // stay at the end of path 2
//                if (pathTimer.milliseconds() >= LINE2_HOLD_MS) {
//                    setPathState(6);
//                }
//                break;
//            case 6: // start path 3, now shooting at the top goal
//                presetIndex = 1;
//                follower.follow(paths.line3);
//                setPathState(7);
//                break;
//            case 7: // path 3: when done, flower DOWN
//                if (!follower.isBusy()) {
//                    setFlowerPosition(FLOWER_DOWN);
//                    setPathState(8);
//                }
//                break;
//            case 8: // shoot
//                shooting = true;
//                if (pathTimer.milliseconds() >= SHOOT_MS) {
//                    setPathState(9);
//                }
//                break;
//            case 9: // start path 4
//                follower.follow(paths.line4);
//                setPathState(10);
//                break;
//            case 10: // path 4: picking up balls from the flower
//                if (!follower.isBusy()) {
//                    setPathState(11);
//                }
//                break;
//            case 11: // start path 5
//                follower.follow(paths.line5);
//                setPathState(12);
//                break;
//            case 12: // path 5: when done, flower UP
//                if (!follower.isBusy()) {
//                    setFlowerPosition(FLOWER_UP);
//                    setPathState(13);
//                }
//                break;
//            case 13: // shoot
//                shooting = true;
//                if (pathTimer.milliseconds() >= SHOOT_MS) {
//                    setPathState(14);
//                }
//                break;
//            case 14: // start path 6 (park)
//                follower.follow(paths.line6);
//                setPathState(15);
//                break;
//            case 15: // path 6: parking
//                if (!follower.isBusy()) {
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                requestOpModeStop();
//                pathState = -1;
//                break;
//        }
//        return pathState;
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.reset();
//    }
//}