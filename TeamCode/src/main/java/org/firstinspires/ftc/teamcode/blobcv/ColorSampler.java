package org.firstinspires.ftc.teamcode.blobcv;

// ═══════════════════════════════════════════════════════════════════════════════
//  LiveBrightnessTuner.java  —  Tune exposure, gain and a BRIGHTNESS range LIVE
//                          with gamepad 1. No redeploying.
//
//  CAMERA: SWYFT USB Camera (OV9281-class global-shutter MONOCHROME sensor).
//    This camera has no color channel, so it can't be tuned by hue/saturation
//    like a normal webcam. Instead this tuner thresholds on raw brightness
//    (0 = black, 255 = white): a bright yellow ball under the robot's own
//    lighting/exposure comes back as a clearly brighter blob than the field
//    around it, so "find pixels brighter than X" does the same job HSV would
//    on a color camera — it just can't tell bright-yellow apart from any other
//    bright-colored object. If the field has other bright/reflective stuff in
//    frame (white lines, glare, bright jerseys) you may need to also narrow
//    EXPOSURE so only the ball is bright enough to cross the threshold.
//    Fixed MJPEG modes only: 160x120 / 320x240 / 352x288 / 640x400 @210fps,
//    640x480 / 800x600 / 1024x768 / 1280x720 / 1280x800 @120fps.
//
//  HOW TO USE
//    1. Run this OpMode and stay in INIT (everything works in INIT).
//    2. Driver Station → three-dot menu → Camera Stream. Tap the image to
//       refresh it after each change (the preview doesn't auto-update).
//    3. Point the camera at a yellow ball so it sits in the green sample box
//       in the middle of the image.
//    4. Use the controls below. Telemetry shows every value and marks the one
//       you're editing with  >> .
//    5. Press X to auto-set the brightness range from the ball in the box.
//       Lower EXPOSURE (dpad left to select it, dpad down to lower) until the
//       field background drops out of range but the ball stays in it.
//    6. When the mask is clean, copy the lines under "COPY INTO BlobChaseTest"
//       into BlobChaseTest. Values are NOT saved when the OpMode stops.
//
//  CONTROLS (gamepad 1)
//    dpad left / right  — pick which value to edit
//    dpad up / down     — raise / lower it
//    hold right bumper  — change in bigger steps (×10)
//    A                  — switch view: NORMAL image  ↔  black/white MASK
//    X                  — auto-sample: set the brightness range from whatever
//                         is in the green box in the middle of the image (± margin)
// ═══════════════════════════════════════════════════════════════════════════════

import android.graphics.Canvas;
import android.util.Size;

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

@TeleOp(name = "LiveBrightnessTuner", group = "Vision")
public class ColorSampler extends LinearOpMode {

    // ── Camera resolution ────────────────────────────────────────────────────
    // SWYFT USB Camera only offers fixed MJPEG modes; 1280x720@120fps per spec.
    private static final int CAMERA_WIDTH  = 1280;
    private static final int CAMERA_HEIGHT = 720;

    // ── Starting values ──────────────────────────────────────────────────────
    private int exposureMs = 5;
    private int gain       = 50;
    private static final int WHITE_BALANCE = 4000;

    // [brightnessLow, brightnessHigh], 0-255. Pixels inside this range = "ball".
    private final int[] brightness = {200, 255};

    // Margin used by the X (auto-sample) button.
    private static final int BRIGHTNESS_MARGIN = 30;

    // ── Editable values, in the order dpad left/right cycles through ─────────
    private static final String[] NAMES = {
            "Exposure (ms)", "Gain", "Brightness low", "Brightness high"
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
        processor.setRange(brightness);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // SWYFT USB Camera is MJPEG-only — it has no raw/YUY2 mode, and
                // VisionPortal defaults to requesting YUY2, which this camera
                // rejects ("doesn't support format YUY2"). Force MJPEG instead.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
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
                processor.setRange(brightness);
                message = "";
            }

            // ── A: toggle normal / mask view ────────────────────────────────
            if (a && !lastA) processor.showMask = !processor.showMask;

            // ── X: auto-sample the green box ────────────────────────────────
            if (x && !lastX) {
                int b = (int) Math.round(processor.getSample());
                brightness[0] = clamp(b - BRIGHTNESS_MARGIN, 0, 255);
                brightness[1] = clamp(b + BRIGHTNESS_MARGIN, 0, 255);
                processor.setRange(brightness);
                message = String.format("Sampled brightness %d", b);
            }

            lastLeft = left; lastRight = right; lastUp = up; lastDown = down;
            lastA = a; lastX = x;

            // ── Telemetry ───────────────────────────────────────────────────
            double live = processor.getSample();
            int[] values = {exposureMs, gain, brightness[0], brightness[1]};

            telemetry.addData("View", processor.showMask ? "MASK (A to switch)" : "NORMAL (A to switch)");
            telemetry.addData("Step", step == 1 ? "1  (hold RB for 10)" : "10");
            telemetry.addLine("---");
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addLine((i == selected ? ">> " : "   ") + NAMES[i] + ": " + values[i]);
            }
            telemetry.addLine("---");
            telemetry.addData("Green box brightness", "%.0f / 255", live);
            telemetry.addData("White (ball) pixels", processor.whitePixels);
            if (!message.isEmpty()) telemetry.addLine(message);
            telemetry.addLine("--- COPY INTO BlobChaseTest ---");
            telemetry.addLine(String.format("EXPOSURE_MS = %d;  GAIN = %d;", exposureMs, gain));
            telemetry.addLine(String.format("BRIGHTNESS_LOW = %d;  BRIGHTNESS_HIGH = %d;", brightness[0], brightness[1]));
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
                int i = selected - 2;   // index into brightness[]
                brightness[i] = clamp(brightness[i] + change, 0, 255);
                // Keep low ≤ high.
                if (brightness[0] > brightness[1]) {
                    if (i == 0) brightness[1] = brightness[0];
                    else        brightness[0] = brightness[1];
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
    //  the average brightness inside a small green box in the center.
    //  Mono-sensor friendly: works on a single grayscale channel instead of HSV,
    //  since the SWYFT USB Camera has no color information to threshold on.
    // ═════════════════════════════════════════════════════════════════════════
    static class TunerProcessor implements VisionProcessor {

        private static final int BOX_SIZE = 20;

        private final Mat grayMat = new Mat();
        private final Mat mask    = new Mat();

        // Shared between the OpMode thread and the camera thread.
        volatile boolean showMask = false;
        volatile int     whitePixels = 0;
        private volatile int lower  = 200;
        private volatile int upper  = 255;
        private volatile double sample = 0;

        void setRange(int[] brightness) {
            lower = brightness[0];
            upper = brightness[1];
        }

        double getSample() { return sample; }

        @Override
        public void init(int width, int height, CameraCalibration calibration) { }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, grayMat, Imgproc.COLOR_RGB2GRAY);

            // Sample the green box (always, in either view).
            int bx = frame.cols() / 2 - BOX_SIZE / 2;
            int by = frame.rows() / 2 - BOX_SIZE / 2;
            Mat region = grayMat.submat(new Rect(bx, by, BOX_SIZE, BOX_SIZE));
            sample = Core.mean(region).val[0];
            region.release();

            // Build the mask from the current brightness range.
            Core.inRange(grayMat, new Scalar(lower), new Scalar(upper), mask);
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