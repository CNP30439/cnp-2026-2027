package org.firstinspires.ftc.teamcode.blobcv;

// ═══════════════════════════════════════════════════════════════════════════════
//  LiveColorTuner.java  —  Tune exposure, gain and HSV range LIVE with gamepad 1.
//                          No redeploying. Replaces ColorSampler + MaskViewer
//                          for tuning (you can keep them, they still work).
//
//  HOW TO USE
//    1. Run this OpMode and stay in INIT (everything works in INIT).
//    2. Driver Station → three-dot menu → Camera Stream. Tap the image to
//       refresh it after each change (the preview doesn't auto-update).
//    3. Use the controls below. Telemetry shows every value and marks the one
//       you're editing with  >> .
//    4. When the mask is clean, copy the lines under "COPY INTO BlobChaseTest"
//       into BlobChaseTest. Values are NOT saved when the OpMode stops.
//
//  CONTROLS (gamepad 1)
//    dpad left / right  — pick which value to edit
//    dpad up / down     — raise / lower it
//    hold right bumper  — change in bigger steps (×10)
//    A                  — switch view: NORMAL image  ↔  black/white MASK
//    X                  — auto-sample: set the HSV range from whatever is in
//                         the green box in the middle of the image (± margins)
//
//  OpenCV HSV RANGES: H 0–180, S 0–255, V 0–255
// ═══════════════════════════════════════════════════════════════════════════════

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "LiveColorTuner", group = "Vision")
public class ColorSampler extends LinearOpMode {

    // ── Starting values ──────────────────────────────────────────────────────
    private int exposureMs = 5;
    private int gain       = 50;
    private static final int WHITE_BALANCE = 4000;

    // [hLow, hHigh, sLow, sHigh, vLow, vHigh]
    private final int[] hsv = {15, 35, 80, 255, 80, 255};

    // Margins used by the X (auto-sample) button.
    private static final int H_MARGIN = 10;
    private static final int S_MARGIN = 40;
    private static final int V_MARGIN = 40;

    // ── Editable values, in the order dpad left/right cycles through ─────────
    private static final String[] NAMES = {
            "Exposure (ms)", "Gain",
            "H low", "H high", "S low", "S high", "V low", "V high"
    };
    private int selected = 0;

    // Camera limits, read from the camera at startup.
    private int minExposure = 1, maxExposure = 100;
    private int minGain     = 0, maxGain     = 255;

    private ExposureControl exposureControl;
    private GainControl     gainControl;

    @Override
    public void runOpMode() {
        TunerProcessor processor = new TunerProcessor();
        processor.setRange(hsv);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();

        // Camera controls only work once the camera is streaming.
        telemetry.addLine("Waiting for camera...");
        telemetry.update();
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        setUpCameraControls(portal);
        applyExposure();
        applyGain();

        boolean lastLeft = false, lastRight = false, lastUp = false, lastDown = false;
        boolean lastA = false, lastX = false;
        String message = "";

        while (opModeInInit() || opModeIsActive()) {

            boolean left  = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean up    = gamepad1.dpad_up;
            boolean down  = gamepad1.dpad_down;
            boolean a     = gamepad1.a;
            boolean x     = gamepad1.x;
            int step      = gamepad1.right_bumper ? 10 : 1;

            // ── Pick which value to edit ────────────────────────────────────
            if (right && !lastRight) selected = (selected + 1) % NAMES.length;
            if (left  && !lastLeft)  selected = (selected - 1 + NAMES.length) % NAMES.length;

            // ── Change it ───────────────────────────────────────────────────
            int change = 0;
            if (up   && !lastUp)   change = +step;
            if (down && !lastDown) change = -step;
            if (change != 0) {
                changeSelected(change);
                processor.setRange(hsv);
                message = "";
            }

            // ── A: toggle normal / mask view ────────────────────────────────
            if (a && !lastA) processor.showMask = !processor.showMask;

            // ── X: auto-sample the green box ────────────────────────────────
            if (x && !lastX) {
                double[] s = processor.getSample();
                int h = (int) Math.round(s[0]);
                int sa = (int) Math.round(s[1]);
                int v = (int) Math.round(s[2]);
                hsv[0] = clamp(h  - H_MARGIN, 0, 180);
                hsv[1] = clamp(h  + H_MARGIN, 0, 180);
                hsv[2] = clamp(sa - S_MARGIN, 0, 255);
                hsv[3] = clamp(sa + S_MARGIN, 0, 255);
                hsv[4] = clamp(v  - V_MARGIN, 0, 255);
                hsv[5] = clamp(v  + V_MARGIN, 0, 255);
                processor.setRange(hsv);
                message = String.format("Sampled H %d S %d V %d", h, sa, v);
            }

            lastLeft = left; lastRight = right; lastUp = up; lastDown = down;
            lastA = a; lastX = x;

            // ── Telemetry ───────────────────────────────────────────────────
            double[] live = processor.getSample();
            int[] values = {exposureMs, gain, hsv[0], hsv[1], hsv[2], hsv[3], hsv[4], hsv[5]};

            telemetry.addData("View", processor.showMask ? "MASK (A to switch)" : "NORMAL (A to switch)");
            telemetry.addData("Step", step == 1 ? "1  (hold RB for 10)" : "10");
            telemetry.addLine("---");
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addLine((i == selected ? ">> " : "   ") + NAMES[i] + ": " + values[i]);
            }
            telemetry.addLine("---");
            telemetry.addData("Green box HSV", "H %.0f  S %.0f  V %.0f", live[0], live[1], live[2]);
            telemetry.addData("White pixels", processor.whitePixels);
            if (!message.isEmpty()) telemetry.addLine(message);
            telemetry.addLine("--- COPY INTO BlobChaseTest ---");
            telemetry.addLine(String.format("EXPOSURE_MS = %d;  GAIN = %d;", exposureMs, gain));
            telemetry.addLine(String.format("new Scalar(%d, %d, %d)", hsv[0], hsv[2], hsv[4]));
            telemetry.addLine(String.format("new Scalar(%d, %d, %d)", hsv[1], hsv[3], hsv[5]));
            telemetry.update();

            sleep(20);
        }

        portal.close();
    }

    // Applies a change to whichever value is selected, clamped to valid limits.
    private void changeSelected(int change) {
        switch (selected) {
            case 0:
                exposureMs = clamp(exposureMs + change, minExposure, maxExposure);
                applyExposure();
                break;
            case 1:
                gain = clamp(gain + change, minGain, maxGain);
                applyGain();
                break;
            default:
                int i   = selected - 2;          // index into hsv[]
                int max = (i < 2) ? 180 : 255;   // hue tops out at 180
                hsv[i]  = clamp(hsv[i] + change, 0, max);
                // Keep low ≤ high.
                int pairLow = i - (i % 2);
                if (hsv[pairLow] > hsv[pairLow + 1]) {
                    if (i % 2 == 0) hsv[pairLow + 1] = hsv[pairLow];
                    else            hsv[pairLow]     = hsv[pairLow + 1];
                }
        }
    }

    private void setUpCameraControls(VisionPortal portal) {
        try {
            exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
            exposureMs  = clamp(exposureMs, minExposure, maxExposure);
        } catch (Exception e) {
            exposureControl = null;
            telemetry.addLine("Exposure not supported: " + e.getMessage());
        }
        try {
            gainControl = portal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
            gain    = clamp(gain, minGain, maxGain);
        } catch (Exception e) {
            gainControl = null;
            telemetry.addLine("Gain not supported: " + e.getMessage());
        }
        try {
            WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
            wb.setWhiteBalanceTemperature(WHITE_BALANCE);
        } catch (Exception e) {
            telemetry.addLine("White balance not supported: " + e.getMessage());
        }
    }

    private void applyExposure() {
        if (exposureControl != null) exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
    }

    private void applyGain() {
        if (gainControl != null) gainControl.setGain(gain);
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Processor: shows either the normal image or the mask, and always samples
    //  the average HSV inside a small green box in the center.
    // ═════════════════════════════════════════════════════════════════════════
    static class TunerProcessor implements VisionProcessor {

        private static final int BOX_SIZE = 20;

        private final Mat hsvMat = new Mat();
        private final Mat mask   = new Mat();

        // Shared between the OpMode thread and the camera thread.
        volatile boolean showMask = false;
        volatile int     whitePixels = 0;
        private volatile Scalar lower  = new Scalar(0, 0, 0);
        private volatile Scalar upper  = new Scalar(180, 255, 255);
        private volatile double[] sample = {0, 0, 0};

        void setRange(int[] hsv) {
            lower = new Scalar(hsv[0], hsv[2], hsv[4]);
            upper = new Scalar(hsv[1], hsv[3], hsv[5]);
        }

        double[] getSample() { return sample.clone(); }

        @Override
        public void init(int width, int height, CameraCalibration calibration) { }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Sample the green box (always, in either view).
            int bx = frame.cols() / 2 - BOX_SIZE / 2;
            int by = frame.rows() / 2 - BOX_SIZE / 2;
            Mat region = hsvMat.submat(new Rect(bx, by, BOX_SIZE, BOX_SIZE));
            Scalar mean = Core.mean(region);
            region.release();
            sample = new double[]{mean.val[0], mean.val[1], mean.val[2]};

            // Build the mask from the current range.
            Core.inRange(hsvMat, lower, upper, mask);
            whitePixels = Core.countNonZero(mask);

            if (showMask) {
                Imgproc.cvtColor(mask, frame, Imgproc.COLOR_GRAY2RGB);
            }

            Imgproc.rectangle(frame,
                    new Point(bx, by),
                    new Point(bx + BOX_SIZE, by + BOX_SIZE),
                    new Scalar(0, 255, 0), 2);

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) { }
    }
}