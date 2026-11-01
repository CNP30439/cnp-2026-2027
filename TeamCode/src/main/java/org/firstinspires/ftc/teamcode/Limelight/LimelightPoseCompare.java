package org.firstinspires.ftc.teamcode.Limelight;

// ═══════════════════════════════════════════════════════════════════════════════
// LimelightPoseCompare.java — TeleOp. Full robot pose from MegaTag, compared
// side by side with Pedro odometry. Pipeline 5, tags 34-37 (biobuzz_4tags_tilt60.fmap).
//
// CHANGE SUMMARY (this revision):
//   + Added MT2_CORR_X / MT2_CORR_Y constants (from MT2 Offset Finder, 9 readings,
//     model: Fixed shift). X = +2.478 in, Y = -1.067 in.
//   + In loop(): after mt2 is computed from the camera, the correction is added to
//     mt2[0] / mt2[1] (null-guarded). Because it's applied right there, EVERYTHING
//     downstream uses the corrected MT2: the "off by" telemetry AND the dpad UP snap.
//   + MT1 is NOT corrected (offset finder was MT2 only).
//   + Everything else is unchanged.
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
// 2. Drive around where the camera can see tags 34-37.
// 3. Compare the "off by" numbers. Good = within ~1-2 in and ~2 deg.
// 4. Stop and hold still for best readings (moving adds blur + lag).
//
// left stick — drive / strafe right stick X — rotate
// X — reset odometry to (7.25, 8, -90)
// dpad UP — snap odometry to the (corrected) MT2 camera pose (only if a tag is seen)
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

    private static final int PIPELINE = 5;
    private static final String LIMELIGHT_NAME = "limelight";
    private static final boolean USE_HELPER = false; // true = use LimelightReset.init() like before

    private boolean lastSwitchOk = false;
    private int switchCount = 0;
    private static final long MAX_STALE_MS = 100;

    // Field center in Pedro inches (map origin)
    private static final double CENTER = 72.0;

    // MT2 correction from MT2 Offset Finder (9 readings, model: Fixed shift)  [NEW]
    private static final double MT2_CORR_X = 2.478;  // inches added to MT2 x
    private static final double MT2_CORR_Y = -1.067; // inches added to MT2 y

    private static final PoseFactory p = PoseFactory.degrees();
    private static final Pose RESET_POSE = p.of(7.25, 8, -90);
    private static final double DRIVE_DEADZONE = 0.05;

    private Follower follower;
    private Limelight3A limelight;
    private boolean lastX = false, lastUp = false;

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
        lastSwitchOk = limelight.pipelineSwitch(PIPELINE);
        switchCount++;
        telemetry.addLine("Pose compare ready. Press X on the reset spot, then drive near tags 34-37.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(RESET_POSE);
        lastSwitchOk = limelight.pipelineSwitch(PIPELINE); // ask again once the OpMode starts
        switchCount++;
    }

    // Keep asking during INIT too, so you can watch it switch before pressing Play
    @Override
    public void init_loop() {
        LLResult r = limelight.getLatestResult();
        enforcePipeline(r);
        telemetry.addData("pipeline in use", r == null ? "no result yet" : String.valueOf(r.getPipelineIndex()));
        telemetry.addData("switch requests sent", switchCount);
        telemetry.addData("last switch accepted", lastSwitchOk);
        telemetry.update();
    }

    // Re-send the pipeline switch if the camera isn't on the right one yet
    private long lastSwitchMs = 0;
    private void enforcePipeline(LLResult result) {
        if (result == null || result.getPipelineIndex() == PIPELINE) return;
        long now = System.currentTimeMillis();
        if (now - lastSwitchMs > 250) { // don't spam it every loop
            lastSwitchOk = limelight.pipelineSwitch(PIPELINE);
            switchCount++;
            lastSwitchMs = now;
        }
    }

    @Override
    public void loop() {
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
        if (result != null && result.isValid() && result.getPipelineIndex() == PIPELINE) {
            stale = result.getStaleness();
            if (stale <= MAX_STALE_MS) {
                tagCount = result.getFiducialResults() == null ? 0 : result.getFiducialResults().size();
                if (tagCount > 0) {
                    mt1 = toPedro(result.getBotpose());
                    mt2 = toPedro(result.getBotpose_MT2());
                }
            }
        }

        // ── MT2 correction (NEW) ─────────────────────────────────────────────
        // Applied once, right after mt2 is built, so telemetry + dpad UP snap both use corrected values.
        if (mt2 != null) {
            mt2[0] += MT2_CORR_X;
            mt2[1] += MT2_CORR_Y;
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
        telemetry.addData("ODOMETRY", "x=%.1f y=%.1f h=%.1f", odoX, odoY, odoH);

        if (tagCount == 0) {
            telemetry.addData("CAMERA", stale > MAX_STALE_MS ? "data too old (%d ms)" : "no tags 34-37 in view", stale);
        } else {
            telemetry.addData("Tags seen", tagCount);
            showRow("MT1 (camera only)", mt1, odoX, odoY, odoH, true);
            showRow("MT2 (camera + odo heading, corrected)", mt2, odoX, odoY, odoH, false);
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
        telemetry.addLine("X = reset odo dpad UP = snap odo to MT2");
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