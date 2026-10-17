package org.firstinspires.ftc.teamcode.Limelight;

// ═══════════════════════════════════════════════════════════════════════════════
//  Tag34Test.java  —  TeleOp. Only looks at AprilTag 34 (RED 0, far left tag).
//
//  Works out the robot's x, y AND heading from tag 34 alone:
//    1. Where the tag is from the robot  (forward / right)   -> which way the tag is
//       from the robot's nose
//    2. Where the robot is from the tag  (in front / to the side) -> where the robot is
//       on the field, because we know where tag 34 is and which way it faces
//    heading = (field direction from robot to tag) - (direction of tag from robot's nose)
//
//  Telemetry: tx, ty, distance, raw numbers, Limelight x/y/heading, Pedro x/y/heading,
//  and the difference.
//
//  CONTROLS (gamepad 1)
//    left stick     — drive / strafe
//    right stick X  — rotate
//    X              — reset Pedro to RED preset 0 pose (7.25, 8, -90)
//    Y              — set Pedro's pose to the Limelight's pose (if tag 34 is seen)
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

@TeleOp(name = "Tag34Test", group = "TeleOp")
public class Tag34Test extends OpMode {

    // ================== TAG 34 ==================
    private static final int    TAG_ID = 34;
    private static final double TAG_X  = 51.0;   // field inches (Pedro)
    private static final double TAG_Y  = 58.0;
    // Which way the FRONT of tag 34 points on the field, in degrees (Pedro angles).
    // Tag 34 faces down the field toward y = 0 (toward the red preset 0 start), so -90.
    // TODO: change if the tag faces a different way.
    private static final double TAG_FACING_DEG = -90.0;
    // ============================================

    private static final PoseFactory p = PoseFactory.degrees();
    private static final Pose START_POSE = p.of(7.25, 8, -90);   // RED preset 0
    private static final double DRIVE_DEADZONE = 0.05;

    private Follower follower;
    private Limelight3A limelight;
    private boolean lastX = false, lastY = false;

    private static double wrapDeg(double deg) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(deg)), Math.cos(Math.toRadians(deg))));
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        limelight = LimelightReset.init(hardwareMap);   // pipeline 5, started
        telemetry.addLine("Tag34Test ready. Robot starts at RED preset 0.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
    }

    @Override
    public void loop() {
        // ── Find tag 34 ─────────────────────────────────────────────────────
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult tag = null;
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == TAG_ID) { tag = fr; break; }
            }
        }

        // ── Work out the Limelight pose from tag 34 ─────────────────────────
        boolean have = false;
        double tx = 0, ty = 0, dist = 0;
        double fwd = 0, right = 0, up = 0;      // tag from robot center (inches)
        double tagSide = 0, tagOut = 0;         // robot from tag (inches)
        double llX = 0, llY = 0, llHeadingDeg = 0;

        if (tag != null) {
            Pose3D tagFromRobot = tag.getTargetPoseRobotSpace();   // X fwd, Y right, Z up
            Pose3D robotFromTag = tag.getRobotPoseTargetSpace();   // X tag-right, Y down, Z out of tag
            if (tagFromRobot != null && robotFromTag != null) {
                Position a = tagFromRobot.getPosition().toUnit(DistanceUnit.INCH);
                Position b = robotFromTag.getPosition().toUnit(DistanceUnit.INCH);

                tx = tag.getTargetXDegrees();
                ty = tag.getTargetYDegrees();
                fwd = a.x;  right = a.y;  up = a.z;
                dist = Math.hypot(fwd, right);   // flat distance, robot center to tag
                tagSide = b.x;  tagOut = b.z;

                // Robot position on the field:
                // start at the tag, go "tagOut" along the way the tag faces,
                // and "tagSide" along the tag's right (as you look at the tag).
                double n = Math.toRadians(TAG_FACING_DEG);
                double outX = Math.cos(n),  outY = Math.sin(n);
                double rgtX = -Math.sin(n), rgtY = Math.cos(n);   // tag's right = facing + 90 deg
                llX = TAG_X + tagOut * outX + tagSide * rgtX;
                llY = TAG_Y + tagOut * outY + tagSide * rgtY;

                // Heading:
                //   field direction from robot to tag
                //   minus the direction of the tag from the robot's nose (left = positive)
                double fieldDirToTag = Math.atan2(TAG_Y - llY, TAG_X - llX);
                double dirFromNose   = Math.atan2(-right, fwd);
                llHeadingDeg = wrapDeg(Math.toDegrees(fieldDirToTag - dirFromNose));

                have = true;
            }
        }

        // ── Resets ──────────────────────────────────────────────────────────
        boolean x = gamepad1.x;
        if (x && !lastX) follower.setPose(START_POSE);
        lastX = x;

        boolean y = gamepad1.y;
        if (y && !lastY && have) follower.setPose(p.of(llX, llY, llHeadingDeg));
        lastY = y;

        // ── Drive ───────────────────────────────────────────────────────────
        double sy = -gamepad1.left_stick_y, sx = gamepad1.left_stick_x, rx = gamepad1.right_stick_x;
        if (Math.abs(sy) < DRIVE_DEADZONE) sy = 0;
        if (Math.abs(sx) < DRIVE_DEADZONE) sx = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;
        follower.manual(sy, -sx, -rx);
        follower.update();

        // ── Telemetry ───────────────────────────────────────────────────────
        Pose pedro = follower.pose();
        double pX = pedro.x(), pY = pedro.y(), pH = wrapDeg(Math.toDegrees(pedro.heading()));

        if (have) {
            telemetry.addData("TAG 34", "tx=%.1f deg  ty=%.1f deg  distance=%.1f in", tx, ty, dist);
            telemetry.addData("tag from robot", "fwd=%.1f  right=%.1f  up=%.1f", fwd, right, up);
            telemetry.addData("robot from tag", "side=%.1f  out=%.1f", tagSide, tagOut);
            telemetry.addLine("---");
            telemetry.addData("LIMELIGHT", "x=%.1f  y=%.1f  heading=%.1f deg", llX, llY, llHeadingDeg);
            telemetry.addData("PEDRO", "x=%.1f  y=%.1f  heading=%.1f deg", pX, pY, pH);
            telemetry.addLine("---");
            telemetry.addData("DIFFERENCE", "dx=%.1f  dy=%.1f  dist=%.1f in  dHeading=%.1f deg",
                    llX - pX, llY - pY, Math.hypot(llX - pX, llY - pY), wrapDeg(llHeadingDeg - pH));
        } else {
            telemetry.addData("TAG 34", "not seen");
            telemetry.addLine("---");
            telemetry.addData("LIMELIGHT", "N/A");
            telemetry.addData("PEDRO", "x=%.1f  y=%.1f  heading=%.1f deg", pX, pY, pH);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}