//
//
///* ============================================================= *
// *                 Turtle Tracer — Auto-Generated                *
// *                                                               *
// *  Version: 2.7.0.                                              *
// *  Copyright (c) 2026 Matthew Allen                             *
// *                                                               *
// *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
// *  Changes will be overwritten when regenerated.                *
// * ============================================================= */
//
//
//package org.firstinspires.ftc.teamcode.Auto;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.Path;
//import static com.pedropathing.api.Paths.curve;
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.api.Paths.path;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.interpolator.Interpolator;
//
//@Autonomous(name = "AutoPath", group = "Autonomous")
//public class parkauto extends OpMode {
//    public Follower follower; // Pathing follower instance
//    private final PoseFactory p = PoseFactory.degrees();
//    private int pathState; // Current autonomous path state (state machine)
//    private ElapsedTime pathTimer; // Timer for path state machine
//    private Paths paths; // Paths defined in the Paths class
//
//
//    @Override
//    public void init() {
//
//
//        follower = Constants.create(hardwareMap);
//        follower.setPose(p.of(28.137, 134.815, 0.000));
//
//        pathTimer = new ElapsedTime();
//        paths = new Paths(follower); // Build paths
//
//    }
//
//    @Override
//    public void loop() {
//        follower.update(); // Update follower
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//
//        // Log values to Panels and Driver Station
//
//    }
//
//
//    public static class Paths {
//        private static final PoseFactory p = PoseFactory.degrees();
//        public Path DriveToShoot;
//
//        public Paths(Follower follower) {
//            DriveToShoot = line(
//                    p.of(28.137, 134.815, 0.0),
//                    p.of(28.137, 108.943, 0.0)
//            ).linear(Math.toRadians(0), Math.toRadians(90));
//        }
//
//
//
//    }
//
//
//
//
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//
//            case 0:
//                follower.follow(paths.DriveToShoot);
//                setPathState(1);
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    setPathState(2);
//                }
//                break;
//            case 2:
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
