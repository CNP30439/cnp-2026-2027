package org.firstinspires.ftc.teamcode.Limelight;

// ═══════════════════════════════════════════════════════════════════════════════
// LimelightPoseCompare.java — TeleOp. Full robot pose from MegaTag, compared
// side by side with Pedro odometry. All 16 tags via 4 switchable pipelines.
//
//   Red 0  = pipeline 5 = tags 34-37
//   Red 1  = pipeline 4 = tags 30-33
//   Blue 0 = pipeline 3 = tags 38-41
//   Blue 1 = pipeline 2 = tags 42-45
//
// CHANGE SUMMARY (this revision):
//   - Removed the MT2 correction (MT2_CORR_X / MT2_CORR_Y). MT2 is now raw.
//   + Left/right bumper cycles the active pipeline (works in INIT and RUN).
//
// The map was built in Pedro inches with the field center at (72, 72), so:
// pedroX = limelightX(in) + 72
// pedroY = limelightY(in) + 72
// heading = limelight yaw (same direction as Pedro)
//
// MT1 = camera only (x, y AND heading from the tags)
// MT2 = camera x, y using ODOMETRY heading (steadier, no flipping)
//
// HOW TO USE (gamepad 1):
// 1. Put robot on the reset spot, press X (odometry = 7.25, 8, -90).
// 2. Pick the pipeline for the tags you're facing (bumpers).
// 3. Drive around where the camera can see those tags.
// 4. Compare the "off by" numbers. Good = within ~1-2 in and ~2 deg.
// 5. Stop and hold still for best readings (moving adds blur + lag).
//
// left stick — drive / strafe right stick X — rotate
// right bumper — next pipeline left bumper — previous pipeline
// X — reset odometry to (7.25, 8, -90)
// dpad UP — snap odometry to the MT2 camera pose (only if a tag is seen)
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

@TeleOp(name = "LimelightPoseCompare", group = "TeleOp")
public class LimelightPoseCompare extends OpMode {

    // ── Pipeline table (index order = bumper cycle order) ────────────────────
    private static final int[]    PIPELINES = {5, 4, 3, 2};
    private static final String[] SET_NAMES = {"Red 0", "Red 1", "Blue 0", "Blue 1"};
    private static final String[] TAG_RANGE = {"34-37", "30-33", "38-41", "42-45"};
    private int currentIdx = 0; // starts on Red 0 (pipeline 5)

    private int pipeline() { return PIPELINES[currentIdx]; }

    private static final String LIMELIGHT_NAME = "orange";
    private static final boolean USE_HELPER = false; // true = use LimelightReset.init() like before

    private boolean lastSwitchOk = false;
    private int switchCount = 0;
    private static final long MAX_STALE_MS = 100;

    // Field center in Pedro inches (map origin)
    private static final double CENTER = 72.0;

    private static final PoseFactory p = PoseFactory.degrees();
    private static final Pose RESET_POSE = p.of(7.25, 8, -90);
    private static final double DRIVE_DEADZONE = 0.05;

    private Follower follower;
    private Limelight3A limelight;
    private boolean lastX = false, lastUp = false;
    private boolean lastLB = false, lastRB = false;

    private static double wrapDeg(double deg) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(deg)), Math.cos(Math.toRadians(deg))));
    }

    /** Converts a Limelight botpose to {pedroX, pedroY, headingDeg}, or null if no real solution. */
    private static double[] toPedro(Pose3D bp) {
        if (bp == null) return null;
        Position pos = bp.getPosition().toUnit(DistanceUnit.INCH);
        // Limelight returns exactly (0,0,0) when it can't solve
        if (pos.x == 0 && pos.y == 0 && pos.z == 0) return null;
        return new double[] {
                pos.x + CENTER,
                pos.y + CENTER,
                wrapDeg(bp.getOrientation().getYaw(AngleUnit.DEGREES))
        };
    }

    /** Handles bumper presses to change the active pipeline. Call every loop (INIT and RUN). */
    private void handlePipelineButtons() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean changed = false;

        if (rb && !lastRB) {
            currentIdx = (currentIdx + 1) % PIPELINES.length;
            changed = true;
        }
        if (lb && !lastLB) {
            currentIdx = (currentIdx - 1 + PIPELINES.length) % PIPELINES.length;
            changed = true;
        }
        lastRB = rb;
        lastLB = lb;

        if (changed) {
            lastSwitchOk = limelight.pipelineSwitch(pipeline());
            switchCount++;
            lastSwitchMs = System.currentTimeMillis();
        }
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(RESET_POSE);
        // Get the camera directly (skips the LimelightReset helper, in case it forces pipeline 0).
        // If the robot config names it something other than "limelight", change LIMELIGHT_NAME.
        limelight = USE_HELPER ? LimelightReset.init(hardwareMap)
                : hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();
        lastSwitchOk = limelight.pipelineSwitch(pipeline());
        switchCount++;
        telemetry.addLine("Pose compare ready. Bumpers = change pipeline. Press X on the reset spot, then drive near tags.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(RESET_POSE);
        lastSwitchOk = limelight.pipelineSwitch(pipeline()); // ask again once the OpMode starts
        switchCount++;
    }

    // Keep asking during INIT too, so you can watch it switch before pressing Play
    @Override
    public void init_loop() {
        handlePipelineButtons();
        LLResult r = limelight.getLatestResult();
        enforcePipeline(r);
        telemetry.addData("SET", "%s (pipeline %d, tags %s)", SET_NAMES[currentIdx], pipeline(), TAG_RANGE[currentIdx]);
        telemetry.addData("pipeline in use", r == null ? "no result yet" : String.valueOf(r.getPipelineIndex()));
        telemetry.addData("switch requests sent", switchCount);
        telemetry.addData("last switch accepted", lastSwitchOk);
        telemetry.update();
    }

    // Re-send the pipeline switch if the camera isn't on the right one yet
    private long lastSwitchMs = 0;
    private void enforcePipeline(LLResult result) {
        if (result == null || result.getPipelineIndex() == pipeline()) return;
        long now = System.currentTimeMillis();
        if (now - lastSwitchMs > 250) { // don't spam it every loop
            lastSwitchOk = limelight.pipelineSwitch(pipeline());
            switchCount++;
            lastSwitchMs = now;
        }
    }

    @Override
    public void loop() {
        // ── Pipeline select ──────────────────────────────────────────────────
        handlePipelineButtons();

        // ── Odometry ─────────────────────────────────────────────────────────
        Pose odo = follower.pose();
        double odoX = odo.x(); // if your Pose uses getX()/getY(), change these two
        double odoY = odo.y();
        double odoH = wrapDeg(Math.toDegrees(odo.heading()));

        // MT2 needs the robot heading every loop (same frame as the map = Pedro)
        limelight.updateRobotOrientation(odoH);

        // ── Camera ───────────────────────────────────────────────────────────
        LLResult result = limelight.getLatestResult();
        enforcePipeline(result);
        double[] mt1 = null, mt2 = null;
        int tagCount = 0;
        long stale = -1;
        if (result != null && result.isValid() && result.getPipelineIndex() == pipeline()) {
            stale = result.getStaleness();
            if (stale <= MAX_STALE_MS) {
                tagCount = result.getFiducialResults() == null ? 0 : result.getFiducialResults().size();
                if (tagCount > 0) {
                    mt1 = toPedro(result.getBotpose());
                    mt2 = toPedro(result.getBotpose_MT2());
                }
            }
        }

        // ── Buttons ──────────────────────────────────────────────────────────
        boolean x = gamepad1.x;
        if (x && !lastX) follower.setPose(RESET_POSE);
        lastX = x;

        boolean up = gamepad1.dpad_up;
        if (up && !lastUp && mt2 != null) follower.setPose(p.of(mt2[0], mt2[1], odoH));
        lastUp = up;

        // ── Drive ────────────────────────────────────────────────────────────
        double stickY = -gamepad1.left_stick_y;
        double stickX = gamepad1.left_stick_x;
        double stickRx = gamepad1.right_stick_x;
        if (Math.abs(stickY) < DRIVE_DEADZONE) stickY = 0;
        if (Math.abs(stickX) < DRIVE_DEADZONE) stickX = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;
        follower.manual(stickY, -stickX, -stickRx);
        follower.update();

        // ── Telemetry ────────────────────────────────────────────────────────
        telemetry.addData("SET", "%s (pipeline %d, tags %s)  [LB/RB to change]",
                SET_NAMES[currentIdx], pipeline(), TAG_RANGE[currentIdx]);
        telemetry.addData("ODOMETRY", "x=%.1f y=%.1f h=%.1f", odoX, odoY, odoH);

        if (tagCount == 0) {
            if (stale > MAX_STALE_MS) {
                telemetry.addData("CAMERA", "data too old (%d ms)", stale);
            } else {
                telemetry.addData("CAMERA", "no tags %s in view", TAG_RANGE[currentIdx]);
            }
        } else {
            telemetry.addData("Tags seen", tagCount);
            showRow("MT1 (camera only)", mt1, odoX, odoY, odoH, true);
            showRow("MT2 (camera + odo heading)", mt2, odoX, odoY, odoH, false);
        }
        // ── Debug: what the camera is actually sending ───────────────────────
        telemetry.addLine("--- debug ---");
        if (result == null) {
            telemetry.addData("result", "NULL (camera not connected / not started)");
        } else {
            telemetry.addData("pipeline in use", result.getPipelineIndex());
            telemetry.addData("switch requests sent", switchCount);
            telemetry.addData("last switch accepted", lastSwitchOk);
            telemetry.addData("valid", result.isValid());
            telemetry.addData("data age (ms)", result.getStaleness());
            telemetry.addData("tags in result",
                    result.getFiducialResults() == null ? 0 : result.getFiducialResults().size());
            telemetry.addData("tx", "%.1f", result.getTx());
        }
        telemetry.addLine("X = reset odo dpad UP = snap odo to MT2 LB/RB = pipeline");
        telemetry.update();
    }

    private void showRow(String name, double[] cam, double ox, double oy, double oh, boolean showHeading) {
        if (cam == null) {
            telemetry.addData(name, "no solution");
            return;
        }
        double dx = cam[0] - ox, dy = cam[1] - oy;
        telemetry.addData(name, "x=%.1f y=%.1f h=%.1f", cam[0], cam[1], cam[2]);
        if (showHeading) {
            telemetry.addData("  off by", "x %.1f y %.1f dist %.1f in h %.1f deg",
                    dx, dy, Math.hypot(dx, dy), wrapDeg(cam[2] - oh));
        } else {
            telemetry.addData("  off by", "x %.1f y %.1f dist %.1f in", dx, dy, Math.hypot(dx, dy));
        }
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}