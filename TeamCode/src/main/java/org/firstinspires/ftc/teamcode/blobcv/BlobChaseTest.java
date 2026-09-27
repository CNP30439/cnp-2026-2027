package org.firstinspires.ftc.teamcode.blobcv;

// ═══════════════════════════════════════════════════════════════════════════════
//  BlobChaseTest.java  —  Hold R1 to auto-chase the biggest Pollen blob.
//
//  CONTROLS (gamepad 1)
//    left stick        — normal mecanum driving
//    right stick X     — rotate
//    right bumper HELD — auto-chase the biggest valid blob
//
//  HOW THE CHASE WORKS (every loop tick, while R1 is held)
//    1. Get all blobs from the camera.
//    2. Keep only ones big enough (MIN_BLOB_AREA) and round enough (MIN_CIRCULARITY).
//    3. Pick the biggest remaining blob.
//    4. TURN:    drive the blob's center toward TARGET_X (image center) proportionally.
//                Robot faces the ball before charging.
//    5. FORWARD: drive toward the ball until its area reaches TARGET_AREA
//                (measured when a ball is at the intake). Slows as it arrives.
//    6. ARRIVED  → switch to a fixed-time blind push, then stop. No blob → stop.
//
//  CHANGE LOG (fix pass)
//    - Removed CAMERA_X_OFFSET_IN / CAMERA_Y_OFFSET_IN / CAMERA_HEIGHT_IN /
//      PIXELS_PER_INCH. That offset math was computing TARGET_X ≈ -42, which is
//      OFF THE EDGE of the 320px-wide image, causing constant one-directional
//      turning and forward power stuck near 0 via the centeredness multiplier.
//      TARGET_X is now just IMAGE_WIDTH / 2.0 (true image center). If your
//      camera isn't mechanically centered on the intake, re-measure: hold a
//      ball at the spot where the robot should drive straight in, read
//      "Blob X" off telemetry, and use that number instead.
//    - Added a TURN_DIRECTION flip switch. If the robot turns away from the
//      ball instead of toward it, flip this one constant instead of touching
//      the math.
//    - Kept the chase-debug telemetry block for future tuning.
//    - Added a FINAL_APPROACH state. The camera's FOV can't actually confirm
//      the ball is at the intake by blob area alone — that close, it can't
//      see it reliably. So once TARGET_AREA is reached, the robot stops
//      trusting vision and just drives straight forward at
//      FINAL_APPROACH_POWER for FINAL_APPROACH_TIME_S seconds, then stops.
//      Tune those two constants to match how far short the vision "arrived"
//      point actually falls from the real intake position.
// ═══════════════════════════════════════════════════════════════════════════════

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "BlobChaseTest", group = "Vision")
public class BlobChaseTest extends LinearOpMode {

    // ── Camera resolution ────────────────────────────────────────────────────
    private static final int IMAGE_WIDTH  = 320;
    private static final int IMAGE_HEIGHT = 240;

    // ── Turn target ──────────────────────────────────────────────────────────
    // The image-x column the ball should sit at when the robot is aimed
    // correctly. Defaults to true image center. If your camera turns out to
    // not be centered on the intake, re-measure: hold a ball at the spot
    // where the robot should drive straight in, read "Blob X" off telemetry,
    // and put that number here instead of IMAGE_WIDTH / 2.0.
    private static final double TARGET_X = IMAGE_WIDTH / 2.0;

    // Flip to -1 if the robot turns AWAY from the ball instead of toward it
    // (e.g. camera feed mirrored, or motor directions don't match the sign
    // convention this file assumes). Leave at +1 to start.
    private static final double TURN_DIRECTION = 1.0;

    // ── Camera settings — keep IDENTICAL to MaskViewer ───────────────────────
    private static final long EXPOSURE_MS   = 5;
    private static final int  GAIN          = 30;
    private static final int  WHITE_BALANCE = 4000;

    // ── Color range — paste final values from MaskViewer / LiveColorTuner ────
    private static final ColorRange TARGET_COLOR = new ColorRange(
            ColorSpace.HSV,
            new Scalar(15,  80,  50),
            new Scalar(35, 255, 255)
    );

    // ── Noise cleanup — keep IDENTICAL to MaskViewer ─────────────────────────
    private static final int ERODE_SIZE  = 10;
    private static final int DILATE_SIZE = 11;

    // ── Blob filters — keep IDENTICAL to MaskViewer ──────────────────────────
    // NOTE: these are both 0 right now, meaning ANY blob (including noise —
    // glare, reflections, stray color) is treated as valid. If the chase ever
    // locks onto something that isn't the ball, raise these. Try something
    // like MIN_BLOB_AREA = 50 and MIN_CIRCULARITY = 0.5 as a starting point
    // and tune from there using the "Blobs seen: X total, Y valid" telemetry.
    private static final double MIN_BLOB_AREA   = 0;     // pixels
    private static final double MIN_CIRCULARITY = 0;

    // ── Chase targets — MEASURE THESE (step 1 of tuning below) ──────────────
    // Put a ball exactly where your intake can grab it (without chasing).
    // Read "Best blob area" from telemetry and put that number here.
    private static final double TARGET_AREA = 3000;   // TUNE THIS

    // ── Chase gains — TUNE THESE (step 2 of tuning below) ───────────────────
    private static final double TURN_GAIN    = 0.005;  // power per pixel off-center
    private static final double FORWARD_GAIN = 0.8;    // power per "fraction of size left"

    private static final double MAX_TURN    = 0.5;
    private static final double MAX_FORWARD = 0.80;

    // Within this many pixels of TARGET_X, turning stops.
    private static final double CENTER_TOLERANCE_PX = 12;

    // ── Final approach (blind drive once TARGET_AREA is reached) ────────────
    // Once vision says we're at TARGET_AREA, stop trusting vision and just
    // drive straight forward for a fixed time at a fixed power.
    private static final double FINAL_APPROACH_POWER  = 0.5;   // TUNE THIS
    private static final double FINAL_APPROACH_TIME_S = 0.7;   // TUNE THIS
    // Trigger for entering final approach: once the proportionally-computed
    // forward speed drops below this (i.e. the robot has naturally slowed
    // because it's close to TARGET_AREA), switch to the blind push instead
    // of relying on the raw area threshold.
    private static final double FINAL_APPROACH_TRIGGER_SPEED = 0.13;   // TUNE THIS

    // ── Drive ────────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private static final double DRIVE_DEADZONE = 0.05;

    // Chase state, tracked across loop iterations.
    private enum ChaseState { IDLE, CHASING, FINAL_APPROACH, DONE }
    private ChaseState chaseFsmState = ChaseState.IDLE;
    private final com.qualcomm.robotcore.util.ElapsedTime finalApproachTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();

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

        // ── Blob locator ─────────────────────────────────────────────────────
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(TARGET_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(ERODE_SIZE)
                .setDilateSize(DILATE_SIZE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .addProcessor(colorLocator)
                .build();

        // Wait for camera to start streaming before locking settings.
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        lockCameraSettings(portal);

        telemetry.addLine("Ready. Hold R1 to chase a blob.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Find the best blob ────────────────────────────────────────
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Blob best = null;
            double bestCirc = 0;
            int roundSeen   = 0;

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                double area = b.getContourArea();
                if (area < MIN_BLOB_AREA) continue;

                double circ = circularity(b.getContour(), area);
                if (circ < MIN_CIRCULARITY) continue;

                roundSeen++;
                if (best == null || area > best.getContourArea()) {
                    best     = b;
                    bestCirc = circ;
                }
            }

            double blobX    = (best != null) ? best.getBoxFit().center.x : 0;
            double blobArea = (best != null) ? best.getContourArea()      : 0;

            double forward, lateral, turn;
            String chaseState;

            // Debug values, filled in below, printed every loop regardless of state.
            double xErrorDebug = 0, turnPreClamp = 0, centerednessDebug = 0, sizeRatioDebug = 0;
            double finalApproachRemainingDebug = 0;

            if (gamepad1.right_bumper) {
                // ════ AUTO-CHASE ════════════════════════════════════════════
                lateral = 0;

                if (chaseFsmState == ChaseState.FINAL_APPROACH) {
                    // Blind: vision is no longer trustworthy this close, just
                    // drive straight for a fixed time then stop.
                    double elapsed  = finalApproachTimer.seconds();
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
                    // Stay stopped until the bumper is released and pressed again.
                    forward    = 0;
                    turn       = 0;
                    chaseState = "DONE — release R1 to reset";

                } else if (best == null) {
                    // No blob and we haven't confirmed arrival yet — just stop
                    // and wait, don't assume arrival.
                    forward       = 0;
                    turn          = 0;
                    chaseFsmState = ChaseState.IDLE;
                    chaseState    = "NO BLOB — stopped";

                } else {
                    // TURN: blob's x vs the target x (image center by default).
                    double xError = blobX - TARGET_X;
                    xErrorDebug = xError;

                    turn = clamp(xError * TURN_GAIN * TURN_DIRECTION, -MAX_TURN, MAX_TURN);
                    turnPreClamp = xError * TURN_GAIN * TURN_DIRECTION;
                    if (Math.abs(xError) < CENTER_TOLERANCE_PX) turn = 0;

                    // FORWARD: how much of the target size is still left.
                    double sizeError = (TARGET_AREA - blobArea) / TARGET_AREA;
                    sizeRatioDebug = blobArea / TARGET_AREA;
                    forward = clamp(sizeError * FORWARD_GAIN, 0, MAX_FORWARD);

                    // Reduce forward while off-center so it turns to face the
                    // ball before driving at it.
                    double centeredness = 1.0 - Math.min(1.0,
                            Math.abs(xError) / (IMAGE_WIDTH / 2.0));
                    centerednessDebug = centeredness;
                    forward *= centeredness;

                    if (forward < FINAL_APPROACH_TRIGGER_SPEED
                            && Math.abs(xError) < CENTER_TOLERANCE_PX) {
                        // Forward speed has naturally throttled down (we're
                        // close per the size-based proportional control) AND
                        // we're centered, so this is a real "arrived," not
                        // just a low speed from being off-center. Vision
                        // can't confirm the last bit of distance anyway, so
                        // finish the approach blind instead of crawling in.
                        chaseFsmState = ChaseState.FINAL_APPROACH;
                        finalApproachTimer.reset();
                        forward    = FINAL_APPROACH_POWER;
                        turn       = 0;
                        chaseState = "SPEED BELOW THRESHOLD — starting final approach";
                    } else {
                        chaseFsmState = ChaseState.CHASING;
                        chaseState    = "CHASING";
                    }
                }

            } else {
                // ════ NORMAL STICK DRIVING ══════════════════════════════════
                forward = -gamepad1.left_stick_y;
                lateral =  gamepad1.left_stick_x;
                turn    =  gamepad1.right_stick_x;

                if (Math.abs(forward) < DRIVE_DEADZONE) forward = 0;
                if (Math.abs(lateral) < DRIVE_DEADZONE) lateral = 0;
                if (Math.abs(turn)    < DRIVE_DEADZONE) turn    = 0;

                chaseState    = "MANUAL";
                chaseFsmState = ChaseState.IDLE;   // reset so next R1 press starts fresh
            }

            drive(forward, lateral, turn);

            // ── Telemetry ─────────────────────────────────────────────────
            telemetry.addData("State",        chaseState);
            telemetry.addData("Blobs seen",   "%d total, %d valid", blobs.size(), roundSeen);
            telemetry.addData("Best blob area",     "%.0f px  (target %.0f)", blobArea, TARGET_AREA);
            telemetry.addData("Best circularity",   "%.2f  (min %.2f)", bestCirc, MIN_CIRCULARITY);
            telemetry.addData("Blob X",        "%.1f  (target %.1f)", blobX, TARGET_X);
            telemetry.addLine("--- Chase debug ---");
            telemetry.addData("Raw xError", "%.1f px", xErrorDebug);
            telemetry.addData("Turn (pre-clamp)", "%.3f", turnPreClamp);
            telemetry.addData("Centeredness", "%.2f (kills forward if near 0)", centerednessDebug);
            telemetry.addData("Size ratio", "%.2f", sizeRatioDebug);
            telemetry.addData("Final approach remaining", "%.2f s", finalApproachRemainingDebug);
            telemetry.addLine("--- Drive ---");
            telemetry.addData("Fwd / Turn",   "%.2f / %.2f", forward, turn);
            telemetry.update();
        }

        drive(0, 0, 0);
        portal.close();
    }

    // 4π × area ÷ perimeter². Circle = 1.0.
    private static double circularity(MatOfPoint contour, double area) {
        MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
        double perimeter = Imgproc.arcLength(c2f, true);
        c2f.release();
        if (perimeter <= 0) return 0;
        return 4 * Math.PI * area / (perimeter * perimeter);
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

    private void lockCameraSettings(VisionPortal portal) {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            sleep(20);
            portal.getCameraControl(GainControl.class).setGain(GAIN);
            sleep(20);
        } catch (Exception e) {
            telemetry.addLine("Exposure/gain not supported: " + e.getMessage());
        }
        try {
            WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
            wb.setWhiteBalanceTemperature(WHITE_BALANCE);
        } catch (Exception e) {
            telemetry.addLine("White balance not supported: " + e.getMessage());
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}