package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Turns Limelight tx / ty into the AprilTag's position on the field floor,
 * measured from the BACK LEFT CORNER of the robot.
 *
 * Robot frame (origin = back left corner):
 *   x = forward (inches)
 *   y = right   (inches)
 *   z = up      (inches)
 *
 * Angle signs:
 *   yaw   + = camera turned to the right (clockwise from above)
 *   pitch + = camera tilted up
 *   roll  + = camera rotated clockwise (seen from behind the camera)
 */
public class LimelightLocator {

    // ================== FILL THESE IN ==================

    // Where the Limelight lens is, measured from the back left corner (inches)
    public static double CAM_X_FORWARD = 0.0;   // TODO: inches forward from back left corner
    public static double CAM_Y_RIGHT   = 0.0;   // TODO: inches to the right of back left corner
    public static double CAM_HEIGHT    = 0.0;   // TODO: inches from floor to lens

    // Which way the Limelight is facing (degrees)
    public static double CAM_YAW_DEG   = 0.0;   // TODO: 0 = straight forward, + = turned right
    public static double CAM_PITCH_DEG = 0.0;   // TODO: + = tilted up, - = tilted down
    public static double CAM_ROLL_DEG  = 0.0;   // TODO: 0 = upright

    // The AprilTag
    public static double TAG_HEIGHT    = 0.0;   // TODO: inches from floor to CENTER of the tag

    // ===================================================

    /** Result of one calculation. All distances in inches. */
    public static class Result {
        public boolean valid = false;
        public double x;               // tag forward of back left corner
        public double y;               // tag right of back left corner
        public double distanceFromCamera;   // straight line along the floor from the camera
        public double distanceFromCorner;   // straight line along the floor from the back left corner
        public double bearingFromCornerDeg; // + = tag is to the right of straight forward
    }

    /**
     * @param txDeg Limelight tx (+ = tag is to the right)
     * @param tyDeg Limelight ty (+ = tag is up)
     */
    public static Result calculate(double txDeg, double tyDeg) {
        Result out = new Result();

        // Ray toward the tag in the camera's own frame: forward = 1, right = tan(tx), up = tan(ty)
        double f = 1.0;
        double r = Math.tan(Math.toRadians(txDeg));
        double u = Math.tan(Math.toRadians(tyDeg));

        // 1) Undo roll (rotate around the forward axis)
        double roll = Math.toRadians(CAM_ROLL_DEG);
        double r1 = r * Math.cos(roll) + u * Math.sin(roll);
        double u1 = -r * Math.sin(roll) + u * Math.cos(roll);
        double f1 = f;

        // 2) Apply pitch (rotate around the right axis)
        double pitch = Math.toRadians(CAM_PITCH_DEG);
        double f2 = f1 * Math.cos(pitch) - u1 * Math.sin(pitch);
        double u2 = f1 * Math.sin(pitch) + u1 * Math.cos(pitch);
        double r2 = r1;

        // 3) Apply yaw (rotate around the up axis)
        double yaw = Math.toRadians(CAM_YAW_DEG);
        double dirForward = f2 * Math.cos(yaw) - r2 * Math.sin(yaw);
        double dirRight   = f2 * Math.sin(yaw) + r2 * Math.cos(yaw);
        double dirUp      = u2;

        // The ray must be going up or down toward the tag's height, not parallel to the floor
        double heightDiff = TAG_HEIGHT - CAM_HEIGHT;
        if (Math.abs(dirUp) < 1e-9) return out;

        double t = heightDiff / dirUp;   // how far along the ray to reach the tag's height
        if (t <= 0) return out;          // tag would be behind the camera, so something is wrong

        // Tag position relative to the camera, then shifted to the back left corner
        out.x = CAM_X_FORWARD + t * dirForward;
        out.y = CAM_Y_RIGHT + t * dirRight;

        out.distanceFromCamera = t * Math.hypot(dirForward, dirRight);
        out.distanceFromCorner = Math.hypot(out.x, out.y);
        out.bearingFromCornerDeg = Math.toDegrees(Math.atan2(out.y, out.x));
        out.valid = true;
        return out;
    }

    /** Convenience: pull tx / ty straight from a Limelight result. */
    public static Result calculate(LLResult result) {
        if (result == null || !result.isValid()) return new Result();
        return calculate(result.getTx(), result.getTy());
    }

    // ---------------------------------------------------------------
    // Example use inside an OpMode:
    //
    //   Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
    //   limelight.pipelineSwitch(0);   // your AprilTag pipeline number
    //   limelight.start();
    //
    //   // in your loop:
    //   LimelightLocator.Result tag = LimelightLocator.calculate(limelight.getLatestResult());
    //   if (tag.valid) {
    //       telemetry.addData("Tag x (fwd)", tag.x);
    //       telemetry.addData("Tag y (right)", tag.y);
    //       telemetry.addData("Dist from corner", tag.distanceFromCorner);
    //       telemetry.addData("Bearing", tag.bearingFromCornerDeg);
    //   }
    // ---------------------------------------------------------------
}