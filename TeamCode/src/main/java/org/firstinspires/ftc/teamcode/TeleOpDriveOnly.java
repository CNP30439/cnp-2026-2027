package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  TeleOpDriveOnly.java  —  Minimal TeleOp: gamepad1 drive only (plain Pedro 3)
//
//  • Left stick / right stick → follower.manual(forward, lateral, turn)
//  • R1 (right_bumper)        → AlignToPose drives the robot to (100, 20, 90°)
//  • Touch a stick mid-align  → alignment cancels, driver gets control back
//
//  Nothing else here (no shooter, hood, intake, turret, sensors, LEDs).
//
//  Drivetrain ownership is decided by AlignToPose.driverHasControl(): this
//  OpMode only calls follower.manual() when that's true, so the driver and
//  Ivy's follow command never fight over the motors.
//
//  Structure per https://pedropathing.com/docs/pathing/guide/setting-up-auto
//  and .../guide/teleop-usage :
//    - follower = Constants.create(hardwareMap)  in init()
//    - follower.manual(fwd, lateral, turn)       for driver control
//    - follower.update()                         every loop, in every mode
//    - Scheduler.execute()                       every loop, to run Ivy commands
//
//  Plain FTC OpMode, not NextFTC — swap the base class once NextFTC is wired in.
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOpDriveOnly", group = "TeleOp")
public class TeleOpDriveOnly extends OpMode {

    private Follower follower;

    // Stick inputs below this are treated as zero, so letting go of the sticks
    // gives true zero power rather than residual stick noise.
    private static final double DRIVE_DEADZONE = 0.05;

    private final AlignToPose alignToPose = new AlignToPose();

    @Override
    public void init() {
        Scheduler.reset();                       // Ivy requires this before use
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        // ── Read the sticks once, up front ────────────────────────────────
        double forward = -gamepad1.left_stick_y; // Controller up = negative Y
        double lateral =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        if (Math.abs(forward) < DRIVE_DEADZONE) forward = 0;
        if (Math.abs(lateral) < DRIVE_DEADZONE) lateral = 0;
        if (Math.abs(turn)    < DRIVE_DEADZONE) turn    = 0;

        // Largest stick magnitude — lets AlignToPose detect the driver grabbing
        // the controls to dodge something, and bail out of the alignment.
        double stickMag = Math.max(Math.abs(forward),
                          Math.max(Math.abs(lateral), Math.abs(turn)));

        // ── Let the state machine decide who owns the drivetrain ──────────
        alignToPose.update(follower, gamepad1.right_bumper, stickMag);

        if (alignToPose.driverHasControl()) {
            follower.manual(forward, lateral, turn);
        }
        // else: Ivy's follow command is driving. Calling manual() here would
        // fight it, so we stay out of the way.

        // ── Both must run every loop tick ─────────────────────────────────
        follower.update();
        Scheduler.execute();

        // ── Telemetry ─────────────────────────────────────────────────────
        Pose pose = follower.pose();
        telemetry.addData("Align State",    alignToPose.state());
        telemetry.addData("Driver Control", alignToPose.driverHasControl());
        telemetry.addData("Follower Mode",  follower.mode());
        telemetry.addLine("---");
        telemetry.addData("Robot X",        "%.2f", pose.x());
        telemetry.addData("Robot Y",        "%.2f", pose.y());
        telemetry.addData("Robot Heading",  "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addLine("---");
        telemetry.addData("dx to target",   "%.2f", alignToPose.dx(follower));
        telemetry.addData("dy to target",   "%.2f", alignToPose.dy(follower));
        telemetry.addData("Distance (in)",  "%.2f", alignToPose.distanceToTarget(follower));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Leave nothing scheduled behind when the OpMode ends.
        Scheduler.reset();
    }
}