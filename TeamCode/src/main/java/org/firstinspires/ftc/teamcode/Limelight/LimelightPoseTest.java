package org.firstinspires.ftc.teamcode.Limelight;

// ═══════════════════════════════════════════════════════════════════════════════
//  LimelightPoseTest.java  —  TeleOp. Drive only (Pedro manual drive), used to check
//                    how close the Limelight's AprilTag position is to Pedro's.
//
//  Telemetry shows:
//    1. Pedro's pose            (x, y, heading)
//    2. Limelight's pose        (x, y, heading, from the AprilTags)
//    3. The difference          (Limelight minus Pedro)
//
//  The Limelight heading needs 2 or more known tags in view at once. With only 1 tag
//  it still gives x and y (using Pedro's heading) and says the heading is unavailable.
//
//  ALLIANCE / PRESET SELECTION  (same as AutoShootDodgeRange)
//    During INIT, gamepad 1:  dpad up = RED,  dpad down = BLUE   (default RED)
//
//  CONTROLS (gamepad 1)
//    left stick        — drive / strafe
//    right stick X     — rotate
//    B (press)         — switch preset 0 <-> 1 (changes which reset pose X uses)
//    X                 — reset Pedro's pose to the reset pose (robot on the reset
//                        spot, facing the same way as the reset heading)
//    Y                 — set Pedro's pose to the Limelight's pose (if it sees tags)
//
//  Needs LimelightReset.java in org.firstinspires.ftc.teamcode.Limelight, with its
//  TODO values filled in. The camera's mounting offset is set in the Limelight's own
//  web UI (3D tab), not in Java — LimelightLocator.java is no longer used/needed.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Limelight.LimelightReset;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

@TeleOp(name = "LimelightPoseTest", group = "TeleOp")
public class LimelightPoseTest extends OpMode {

    // ── Drive ────────────────────────────────────────────────────────────────
    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    private static final PoseFactory p = PoseFactory.degrees();

    // ── Limelight ────────────────────────────────────────────────────────────
    private Limelight3A limelight;

    // ── Alliance + preset reset poses (copied from AutoShootDodgeRange) ──────
    private enum Alliance { RED, BLUE }

    public static final Pose[] RED_RESET_POSES = {
            p.of(7.25, 8, -90),     // RED preset 0 reset pose
            p.of(7.25, 136, 90)     // RED preset 1 reset pose
    };
    public static final Pose[] BLUE_RESET_POSES = {
            p.of(135.25, 8, -90),   // BLUE preset 0 reset pose
            p.of(135.25, 136, 90)   // BLUE preset 1 reset pose
    };

    private Alliance alliance    = Alliance.RED;
    private int      presetIndex = 0;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    /** Reset pose (X button) for the chosen alliance + preset. */
    private Pose originPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    /** Turns an angle difference into the range -180 to 180 degrees. */
    private static double wrapDeg(double deg) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(deg)), Math.cos(Math.toRadians(deg))));
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        alliance    = Alliance.RED;
        presetIndex = 0;
        follower.setPose(originPose());

        // Limelight on the AprilTag pipeline (pipeline 5), started
        limelight = LimelightReset.init(hardwareMap);

        telemetry.addLine("Initialized — choose side: gamepad1 dpad UP = RED, DOWN = BLUE");
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up)   alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;

        // Keep the starting pose in sync with the chosen side.
        follower.setPose(originPose());

        Pose o = originPose();
        telemetry.addLine("Gamepad 1:  dpad UP = RED   |   dpad DOWN = BLUE");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose", "x=%.1f  y=%.1f  heading=%.0f deg",
                o.x(), o.y(), Math.toDegrees(o.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(originPose());
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {

        // ── Preset toggle (gamepad 1 B) ──────────────────────────────────────
        boolean b = gamepad1.b;
        if (b && !lastB) presetIndex = 1 - presetIndex;
        lastB = b;

        // ── Limelight pose (worked out once per loop) ────────────────────────
        LLResult result = limelight.getLatestResult();
        double pedroHeadingRad = follower.pose().heading();

        double[] full = LimelightReset.estimatePose(result);            // x, y, heading, tags
        double[] posOnly = (full == null)
                ? LimelightReset.estimate(result, pedroHeadingRad)      // x, y, tags (Pedro heading)
                : null;

        // ── Resets ───────────────────────────────────────────────────────────
        // X: reset to the reset pose
        boolean x = gamepad1.x;
        if (x && !lastX) {
            follower.setPose(originPose());
        }
        lastX = x;

        // Y: set Pedro's pose to the Limelight's pose
        boolean y = gamepad1.y;
        if (y && !lastY) {
            if (full != null) {
                follower.setPose(p.of(full[0], full[1], Math.toDegrees(full[2])));
            } else if (posOnly != null) {
                follower.setPose(p.of(posOnly[0], posOnly[1], Math.toDegrees(pedroHeadingRad)));
            }
        }
        lastY = y;

        // ── Drive ────────────────────────────────────────────────────────────
        double stickY  = -gamepad1.left_stick_y;
        double stickX  =  gamepad1.left_stick_x;
        double stickRx =  gamepad1.right_stick_x;

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        // Pedro: +lateral = left, +turn = counterclockwise. Sticks are
        // positive to the right, so both are negated.
        follower.manual(stickY, -stickX, -stickRx);
        follower.update();

        // ── Telemetry ────────────────────────────────────────────────────────
        Pose pedro = follower.pose();
        double pedroX = pedro.x();
        double pedroY = pedro.y();
        double pedroHeadingDeg = Math.toDegrees(pedro.heading());

        telemetry.addData("Alliance / preset", "%s  #%d", alliance, presetIndex);
        telemetry.addLine("---");

        if (full != null) {
            double lx = full[0], ly = full[1], lh = Math.toDegrees(full[2]);
            telemetry.addData("LIMELIGHT", "x=%.1f  y=%.1f  heading=%.1f deg  (%d tags)",
                    lx, ly, lh, (int) full[3]);
            telemetry.addData("PEDRO", "x=%.1f  y=%.1f  heading=%.1f deg", pedroX, pedroY, pedroHeadingDeg);
            telemetry.addLine("---");
            telemetry.addData("DIFFERENCE (Limelight - Pedro)", "dx=%.1f  dy=%.1f  dist=%.1f in  dHeading=%.1f deg",
                    lx - pedroX, ly - pedroY, Math.hypot(lx - pedroX, ly - pedroY),
                    wrapDeg(lh - pedroHeadingDeg));
        } else if (posOnly != null) {
            double lx = posOnly[0], ly = posOnly[1];
            telemetry.addData("LIMELIGHT", "x=%.1f  y=%.1f  heading=N/A (need 2+ tags)  (%d tag)",
                    lx, ly, (int) posOnly[2]);
            telemetry.addData("PEDRO", "x=%.1f  y=%.1f  heading=%.1f deg", pedroX, pedroY, pedroHeadingDeg);
            telemetry.addLine("---");
            telemetry.addData("DIFFERENCE (Limelight - Pedro)", "dx=%.1f  dy=%.1f  dist=%.1f in  dHeading=N/A",
                    lx - pedroX, ly - pedroY, Math.hypot(lx - pedroX, ly - pedroY));
        } else {
            telemetry.addData("LIMELIGHT", "no known tags in view");
            telemetry.addData("PEDRO", "x=%.1f  y=%.1f  heading=%.1f deg", pedroX, pedroY, pedroHeadingDeg);
            telemetry.addLine("---");
            telemetry.addData("DIFFERENCE (Limelight - Pedro)", "N/A");
        }

        // ── Raw tag numbers (for checking the camera setup) ─────────────────
        // Each tag's position from the robot center, in inches.
        // Camera faces BACK, so a tag behind the robot should show a NEGATIVE fwd.
        telemetry.addLine("--- raw tags (inches from robot center) ---");
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                double[] t = LimelightReset.tagRobotSpaceInches(fr);
                if (t == null) continue;
                telemetry.addData("tag " + fr.getFiducialId(), "fwd=%.1f  right=%.1f  up=%.1f",
                        t[0], t[1], t[2]);
            }
        }

        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  stop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}