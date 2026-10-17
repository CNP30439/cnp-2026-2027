package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Works out where the robot CENTER is on the field from the AprilTags the Limelight sees,
 * so you can reset Pedro's pose without driving to the reset spot.
 *
 * The camera's mounting offset (position + angle relative to the robot) is set in the
 * Limelight's own web UI (Settings -> 3D tab / Robot-space camera offset), NOT in this file.
 * Because that offset is already known to the Limelight, every tag it sees comes back
 * through getTargetPoseRobotSpace() as "where is this tag relative to the robot CENTER" --
 * no camera trig needed here anymore (LimelightLocator.java is no longer used).
 *
 * Every tag it sees that is in the TAGS table gives its own estimate, and they are averaged.
 * Tag IDs are unique, so one table covers red and blue.
 *
 * estimate() does NOT work out heading -- pass in Pedro's current heading (radians) and
 * keep that heading when you reset. estimatePose() DOES work out heading, but needs 2+
 * known tags in view at once.
 *
 * Limelight robot space (from the Limelight docs): X = forward, Y = right, Z = up.
 * The camera offset typed into the Limelight web page must be in METERS, with yaw 180
 * because the camera faces the back of the robot.
 */
public class LimelightReset {

    // ================== FILL THESE IN ==================

    // Limelight pipeline that is set up for AprilTags, and the name of the
    // Limelight in the Robot Configuration on the Driver Hub.
    public static final int    TAG_PIPELINE   = 5;
    public static final String LIMELIGHT_NAME = "orange";   // TODO: change if yours is named differently

    // Leave these at 1.0. Only flip one to -1.0 if the raw TAG telemetry check shows that
    // axis backwards (see the test steps in LimelightPoseTest).
    public static double RIGHT_SIGN   = 1.0;
    public static double FORWARD_SIGN = 1.0;

    // One row per AprilTag: { id, x, y } -- field inches, Pedro's coordinates.
    // Rows with id -1 are ignored.
    //
    // RED 0   (left-to-right): 34(51,58)   35(55,58)   36(60,58)   37(64,58)
    // RED 1   (left-to-right): 33(51,83.5) 32(55,83.5) 31(60,83.5) 30(64,83.5)
    // BLUE 0  (left-to-right): 38(78,58)   39(83,58)   40(88,58)   41(93,58)
    // BLUE 1  (left-to-right): 45(78,83.5) 44(83,83.5) 43(88,83.5) 42(93,83.5)
    public static final double[][] TAGS = {
            { 34, 51.0, 58.0 },   // RED 0
            { 35, 55.0, 58.0 },   // RED 0
            { 36, 60.0, 58.0 },   // RED 0
            { 37, 64.0, 58.0 },   // RED 0

            { 33, 51.0, 83.5 },   // RED 1
            { 32, 55.0, 83.5 },   // RED 1
            { 31, 60.0, 83.5 },   // RED 1
            { 30, 64.0, 83.5 },   // RED 1

            { 38, 78.0, 58.0 },   // BLUE 0
            { 39, 83.0, 58.0 },   // BLUE 0
            { 40, 88.0, 58.0 },   // BLUE 0
            { 41, 93.0, 58.0 },   // BLUE 0

            { 45, 78.0, 83.5 },   // BLUE 1
            { 44, 83.0, 83.5 },   // BLUE 1
            { 43, 88.0, 83.5 },   // BLUE 1
            { 42, 93.0, 83.5 },   // BLUE 1
    };

    // ===================================================

    /**
     * Gets the Limelight, switches it to the AprilTag pipeline (TAG_PIPELINE) and starts it.
     * Call this once in init().
     */
    public static Limelight3A init(HardwareMap hardwareMap) {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(TAG_PIPELINE);
        limelight.start();
        return limelight;
    }

    /** Returns the table row for this tag id, or null if it isn't in the table. */
    private static double[] findTag(int id) {
        for (double[] row : TAGS) {
            if ((int) row[0] == id && id >= 0) return row;
        }
        return null;
    }

    /**
     * Tag position relative to the robot CENTER, in inches: { forward, right, up }.
     * Limelight robot space: X = forward, Y = right, Z = up.
     * toUnit(INCH) converts from whatever unit the library uses (meters), so no hand conversion.
     */
    public static double[] tagRobotSpaceInches(LLResultTypes.FiducialResult fr) {
        Pose3D pose = fr.getTargetPoseRobotSpace();
        if (pose == null) return null;
        Position pos = pose.getPosition().toUnit(DistanceUnit.INCH);
        return new double[]{ pos.x * FORWARD_SIGN, pos.y * RIGHT_SIGN, pos.z };
    }

    /** Same as above, but returned as { right, forward } for the math below. */
    private static double[] robotSpaceInches(LLResultTypes.FiducialResult fr) {
        double[] t = tagRobotSpaceInches(fr);
        if (t == null) return null;
        return new double[]{ t[1], t[0] };
    }

    /**
     * @param result      latest Limelight result
     * @param headingRad  Pedro's current heading, radians (follower.pose().heading())
     * @return { x, y, numberOfTagsUsed } for the robot center, or null if no known tag was seen
     */
    public static double[] estimate(LLResult result, double headingRad) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> seen = result.getFiducialResults();
        if (seen == null || seen.isEmpty()) return null;

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double sumX = 0, sumY = 0;
        int used = 0;

        for (LLResultTypes.FiducialResult fr : seen) {
            double[] tag = findTag(fr.getFiducialId());
            if (tag == null) continue;

            double[] rs = robotSpaceInches(fr);
            if (rs == null) continue;
            double r = rs[0];   // right
            double f = rs[1];   // forward

            // In field coordinates, forward = (cos, sin) and right = (sin, -cos).
            // tagField = robotCenter + f*forward + r*right, so work backwards:
            double robotX = tag[1] - f * cos - r * sin;
            double robotY = tag[2] - f * sin + r * cos;

            sumX += robotX;
            sumY += robotY;
            used++;
        }

        if (used == 0) return null;
        return new double[]{ sumX / used, sumY / used, used };
    }

    /**
     * Works out x, y AND heading from the tags alone. Needs at least 2 known tags in view
     * at the same time (one tag can give a position but not which way the robot faces).
     *
     * It lines up the tags as the robot sees them with where they really are on the field,
     * and finds the turn + shift that fits best across all of them. Tags that are far apart
     * give a better heading than tags that are close together.
     *
     * @return { x, y, headingRad, numberOfTagsUsed } for the robot center, or null
     */
    public static double[] estimatePose(LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> seen = result.getFiducialResults();
        if (seen == null || seen.size() < 2) return null;

        // q = tag as the robot sees it (x = forward, y = LEFT, so it turns the same way as heading)
        // w = tag where it really is on the field
        double[] qx = new double[seen.size()], qy = new double[seen.size()];
        double[] wx = new double[seen.size()], wy = new double[seen.size()];
        int n = 0;

        for (LLResultTypes.FiducialResult fr : seen) {
            double[] tag = findTag(fr.getFiducialId());
            if (tag == null) continue;

            double[] rs = robotSpaceInches(fr);
            if (rs == null) continue;

            qx[n] = rs[1];     // forward
            qy[n] = -rs[0];    // right -> left
            wx[n] = tag[1];
            wy[n] = tag[2];
            n++;
        }
        if (n < 2) return null;

        // Averages (centers) of each set of points
        double mqx = 0, mqy = 0, mwx = 0, mwy = 0;
        for (int i = 0; i < n; i++) { mqx += qx[i]; mqy += qy[i]; mwx += wx[i]; mwy += wy[i]; }
        mqx /= n; mqy /= n; mwx /= n; mwy /= n;

        // Best turn that lines the robot's view up with the field
        double sCross = 0, sDot = 0;
        for (int i = 0; i < n; i++) {
            double ax = qx[i] - mqx, ay = qy[i] - mqy;
            double bx = wx[i] - mwx, by = wy[i] - mwy;
            sCross += ax * by - ay * bx;
            sDot   += ax * bx + ay * by;
        }
        if (Math.hypot(sCross, sDot) < 1e-6) return null;   // tags all in the same spot
        double heading = Math.atan2(sCross, sDot);

        // Robot center = field center of the tags, minus the robot-view center turned by the heading
        double c = Math.cos(heading), s = Math.sin(heading);
        double x = mwx - (mqx * c - mqy * s);
        double y = mwy - (mqx * s + mqy * c);

        return new double[]{ x, y, heading, n };
    }

    // ---------------------------------------------------------------
    // Example use in AutoShootDodgeRange:
    //
    //   private Limelight3A limelight;       // with the other fields
    //   private boolean lastY = false;
    //
    //   in init():
    //   limelight = LimelightReset.init(hardwareMap);   // pipeline 5, started
    //
    //   in stop():
    //   limelight.stop();
    //
    //   in loop(), e.g. on gamepad 1 Y:
    //   boolean y = gamepad1.y;
    //   if (y && !lastY) {
    //       double h = follower.pose().heading();
    //       double[] est = LimelightReset.estimate(limelight.getLatestResult(), h);
    //       if (est != null) {
    //           follower.setPose(p.of(est[0], est[1], Math.toDegrees(h)));
    //       }
    //   }
    //   lastY = y;
    //
    // Tip: do it while the robot is sitting still, so the camera frame
    // matches where the robot really is.
    // ---------------------------------------------------------------
}