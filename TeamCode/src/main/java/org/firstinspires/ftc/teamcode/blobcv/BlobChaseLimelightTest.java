package org.firstinspires.ftc.teamcode.blobcv;

// ═══════════════════════════════════════════════════════════════════════════════
//  BlobChaseLimelightTest.java  —  Hold R1 to auto-chase the biggest Pollen blob,
//                                   using the Limelight 3A's onboard color pipeline
//                                   instead of the webcam + OpenCV blob detector.
//
//  WHAT CHANGED FROM BlobChaseTest.java
//    - No more VisionPortal / ColorBlobLocatorProcessor / OpenCV. The Limelight
//      does the HSV threshold, erode/dilate, area filter, and circularity
//      ("fullness") filter itself, in its own pipeline (set up in the web UI at
//      limelight.local:5801). Set that pipeline's Target Sort to "Largest" so
//      tx/ty/ta already describe the single biggest valid blob — that replaces
//      the `best` blob-picking loop entirely.
//    - blobX/TARGET_X (pixels) → tx (degrees, already relative to the crosshair,
//      which defaults to image center). So the old "xError = blobX - TARGET_X"
//      collapses to just "xError = tx".
//    - blobArea/TARGET_AREA (pixels) → ta (percent of image, 0-100). Re-measure
//      TARGET_TA by holding a ball at the intake and reading ta live in the web
//      UI or telemetry — it will NOT be the same number as the old TARGET_AREA.
//    - TURN_GAIN, FORWARD_GAIN, FINAL_APPROACH_TRIGGER_SPEED all need to be
//      re-tuned from scratch since the units of the input changed (px → deg,
//      px² → %). Start low and raise them, same as before.
//    - Assumes pipeline 0 = your existing AprilTag pipeline, pipeline
//      BALL_PIPELINE_INDEX = the new color pipeline. Switches to the ball
//      pipeline once when R1 is first pressed, and back to AprilTag on release.
//
//  CONTROLS (gamepad 1)
//    left stick        — normal mecanum driving
//    right stick X      — rotate
//    right bumper HELD — auto-chase the biggest valid blob
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BlobChaseLimelightTest", group = "Vision")
public class BlobChaseLimelightTest extends LinearOpMode {

    // ── Pipelines ────────────────────────────────────────────────────────────
    private static final int APRILTAG_PIPELINE = 0;   // your existing AprilTag pipeline
    private static final int BALL_PIPELINE     = 8;   // Color Detector pipeline

    // ── Chase targets — MEASURE THESE in the web UI first (step 7 of setup) ──
    private static final double TARGET_TA = 8.0;   // TUNE THIS — % of image ta reads when ball is at the intake

    // ── Chase gains — TUNE THESE (start low, raise gradually) ────────────────
    private static final double TURN_GAIN    = 0.04;   // power per degree of tx off-center
    private static final double FORWARD_GAIN = 0.15;   // power per "fraction of size left" (in ta units now)

    private static final double MAX_TURN    = 0.5;
    private static final double MAX_FORWARD = 0.80;

    // Within this many degrees of tx = 0, turning stops.
    private static final double CENTER_TOLERANCE_DEG = 2.0;

    // Roughly half the Limelight 3A's horizontal FOV, for the centeredness falloff below.
    private static final double HALF_FOV_DEG = 27.0;

    // ── Final approach (blind drive once TARGET_TA is reached) ───────────────
    private static final double FINAL_APPROACH_POWER  = 0.5;   // TUNE THIS
    private static final double FINAL_APPROACH_TIME_S = 0.7;   // TUNE THIS
    private static final double FINAL_APPROACH_TRIGGER_SPEED = 0.13;   // TUNE THIS

    // ── Drive ────────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private static final double DRIVE_DEADZONE = 0.05;

    private Limelight3A limelight;

    private enum ChaseState { IDLE, CHASING, FINAL_APPROACH, DONE }
    private ChaseState chaseFsmState = ChaseState.IDLE;
    private final com.qualcomm.robotcore.util.ElapsedTime finalApproachTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

    // Tracks whether we've already told the Limelight to switch pipelines this button-press.
    private boolean wasChasingLastLoop = false;

    @Override
    public void runOpMode() {

        // ── Motors ───────────────────────────────────────────────────────────
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

        // ── Limelight ────────────────────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "orange");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(BALL_PIPELINE);
        limelight.start();

        telemetry.addLine("Ready. Hold R1 to chase a blob.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double forward, lateral, turn;
            String chaseState;

            double xErrorDebug = 0, turnPreClamp = 0, centerednessDebug = 0, taRatioDebug = 0;
            double finalApproachRemainingDebug = 0;
            boolean sawTarget = false;

            if (gamepad1.right_bumper) {
                // Switch to the ball pipeline once, the moment R1 is first pressed.
                if (!wasChasingLastLoop) {
                    limelight.pipelineSwitch(BALL_PIPELINE);
                }
                wasChasingLastLoop = true;

                lateral = 0;

                LLResult result = limelight.getLatestResult();
                boolean haveTarget = (result != null) && result.isValid();
                double tx = haveTarget ? result.getTx() : 0;
                double ta = haveTarget ? result.getTa() : 0;
                sawTarget = haveTarget;

                // ── DIAGNOSTICS — remove once tracking works ────────────────
                telemetry.addData("DBG result==null", result == null);
                if (result != null) {
                    telemetry.addData("DBG result.isValid()", result.isValid());
                    telemetry.addData("DBG result.getPipelineIndex()", result.getPipelineIndex());
                    telemetry.addData("DBG result.getStaleness() ms", result.getStaleness());
                    telemetry.addData("DBG raw tx/ty/ta (any validity)",
                            "%.2f / %.2f / %.3f", result.getTx(), result.getTy(), result.getTa());
                }
                telemetry.addData("DBG limelight.getStatus() pipeline", limelight.getStatus().getPipelineIndex());
                // ──────────────────────────────────────────────────────────

                if (chaseFsmState == ChaseState.FINAL_APPROACH) {
                    double elapsed   = finalApproachTimer.seconds();
                    double remaining = FINAL_APPROACH_TIME_S - elapsed;
                    finalApproachRemainingDebug = Math.max(0, remaining);

                    if (remaining <= 0) {
                        forward       = 0;
                        turn          = 0;
                        chaseFsmState = ChaseState.DONE;
                        chaseState    = "DONE — final approach complete";
                    } else {
                        forward    = FINAL_APPROACH_POWER;
                        turn       = 0;
                        chaseState = "FINAL APPROACH (blind)";
                    }

                } else if (chaseFsmState == ChaseState.DONE) {
                    forward    = 0;
                    turn       = 0;
                    chaseState = "DONE — release R1 to reset";

                } else if (!haveTarget) {
                    forward       = 0;
                    turn          = 0;
                    chaseFsmState = ChaseState.IDLE;
                    chaseState    = "NO TARGET — stopped";

                } else {
                    // TURN: tx is already the degrees-off-center error.
                    double xError = tx;
                    xErrorDebug = xError;

                    turn = clamp(xError * TURN_GAIN, -MAX_TURN, MAX_TURN);
                    turnPreClamp = xError * TURN_GAIN;
                    if (Math.abs(xError) < CENTER_TOLERANCE_DEG) turn = 0;

                    // FORWARD: how much of the target size (ta) is still left.
                    double sizeError = (TARGET_TA - ta) / TARGET_TA;
                    taRatioDebug = ta / TARGET_TA;
                    forward = clamp(sizeError * FORWARD_GAIN, 0, MAX_FORWARD);

                    double centeredness = 1.0 - Math.min(1.0, Math.abs(xError) / HALF_FOV_DEG);
                    centerednessDebug = centeredness;
                    forward *= centeredness;

                    if (forward < FINAL_APPROACH_TRIGGER_SPEED
                            && Math.abs(xError) < CENTER_TOLERANCE_DEG) {
                        chaseFsmState = ChaseState.FINAL_APPROACH;
                        finalApproachTimer.reset();
                        forward    = FINAL_APPROACH_POWER;
                        turn       = 0;
                        chaseState = "TA THRESHOLD — starting final approach";
                    } else {
                        chaseFsmState = ChaseState.CHASING;
                        chaseState    = "CHASING";
                    }
                }

            } else {
                if (wasChasingLastLoop) {
                    limelight.pipelineSwitch(APRILTAG_PIPELINE);
                }
                wasChasingLastLoop = false;

                forward = -gamepad1.left_stick_y;
                lateral =  gamepad1.left_stick_x;
                turn    =  gamepad1.right_stick_x;

                if (Math.abs(forward) < DRIVE_DEADZONE) forward = 0;
                if (Math.abs(lateral) < DRIVE_DEADZONE) lateral = 0;
                if (Math.abs(turn)    < DRIVE_DEADZONE) turn    = 0;

                chaseState    = "MANUAL";
                chaseFsmState = ChaseState.IDLE;
            }

            drive(forward, lateral, turn);

            // ── Telemetry ─────────────────────────────────────────────────
            telemetry.addData("State",       chaseState);
            telemetry.addData("Have target", sawTarget);
            telemetry.addData("TA",          "%.2f%%  (target %.2f%%)", taRatioDebug * TARGET_TA, TARGET_TA);
            telemetry.addLine("--- Chase debug ---");
            telemetry.addData("Raw tx", "%.1f deg", xErrorDebug);
            telemetry.addData("Turn (pre-clamp)", "%.3f", turnPreClamp);
            telemetry.addData("Centeredness", "%.2f (kills forward if near 0)", centerednessDebug);
            telemetry.addData("TA ratio", "%.2f", taRatioDebug);
            telemetry.addData("Final approach remaining", "%.2f s", finalApproachRemainingDebug);
            telemetry.addLine("--- Drive ---");
            telemetry.addData("Fwd / Turn", "%.2f / %.2f", forward, turn);
            telemetry.update();
        }

        drive(0, 0, 0);
        limelight.stop();
    }

    // Standard mecanum mixing.
    private void drive(double forward, double lateral, double turn) {
        double fl = forward + lateral + turn;
        double fr = forward - lateral - turn;
        double bl = forward - lateral + turn;
        double br = forward + lateral - turn;

        double maxPow = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br))));
        if (maxPow > 1.0) { fl /= maxPow; fr /= maxPow; bl /= maxPow; br /= maxPow; }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}