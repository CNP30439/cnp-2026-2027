package org.firstinspires.ftc.teamcode.blobcv;

// ═══════════════════════════════════════════════════════════════════════════════
//  MaskViewer.java  —  Shows the black/white "threshold mask" on the camera stream
//
//  White = pixels inside your HSV color range AND big enough AND round enough
//  Red   = right color and big enough, but NOT round enough (rejected)
//  Black = everything else
//
//  Each blob is labelled with its pixel count and circularity score so you
//  can pick good values for MIN_BLOB_AREA and MIN_CIRCULARITY.
//
//  HOW TO USE
//    1. Run this OpMode and stay in INIT.
//    2. Driver Station → three-dot menu → Camera Stream. Tap to refresh.
//    3. Point the camera at a Pollen. It should be white, background black.
//    4. Similar-colored non-round objects should turn red (rejected).
//    5. Copy your final numbers into BlobChaseTest.
//
//  OpenCV HSV RANGES (8-bit images):
//    H (hue, which color)       0 – 180   (NOT 0–360)
//    S (saturation, how vivid)  0 – 255
//    V (value, how bright)      0 – 255
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
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "MaskViewer", group = "Vision")
public class MaskViewer extends LinearOpMode {

    // ── Tune these ──────────────────────────────────────────────────────────
    private static final Scalar LOWER_HSV = new Scalar(15,  80,  50);
    private static final Scalar UPPER_HSV = new Scalar(35, 255, 255);

    // Camera settings — keep IDENTICAL to BlobChaseTest.
    private static final long EXPOSURE_MS   = 5;
    private static final int  GAIN          = 30;
    private static final int  WHITE_BALANCE = 4000;

    // Noise cleanup — keep IDENTICAL to BlobChaseTest.
    private static final int ERODE_SIZE  = 10;//10
    private static final int DILATE_SIZE = 11;//12

    // Size filter — blobs with fewer pixels than this are erased.
    private static final double MIN_BLOB_AREA = 0;

    // Shape filter — 1.0 = perfect circle, 0.0 = not round at all.
    // Useful range for balls: 0.4 (loose) to 0.7 (strict).
    private static final double MIN_CIRCULARITY = 0.5;

    @Override
    public void runOpMode() {
        MaskProcessor maskProcessor = new MaskProcessor();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(maskProcessor)
                .build();

        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        lockCameraSettings(portal);

        while (opModeInInit() || opModeIsActive()) {
            telemetry.addLine("Open Camera Stream to see the mask");
            telemetry.addData("Lower HSV", LOWER_HSV);
            telemetry.addData("Upper HSV", UPPER_HSV);
            telemetry.addData("Min blob area",    "%.0f px", MIN_BLOB_AREA);
            telemetry.addData("Min circularity",  "%.2f", MIN_CIRCULARITY);
            telemetry.addLine("WHITE = kept   RED = rejected (not round)");
            telemetry.addData("Kept blobs",       maskProcessor.keptBlobs);
            telemetry.addData("Rejected blobs",   maskProcessor.rejectedBlobs);
            telemetry.addData("Biggest kept",     "%.0f px", maskProcessor.biggestArea);
            telemetry.update();
            sleep(50);
        }

        portal.close();
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

    static class MaskProcessor implements VisionProcessor {

        private final Mat hsv  = new Mat();
        private final Mat mask = new Mat();

        volatile int    keptBlobs     = 0;
        volatile int    rejectedBlobs = 0;
        volatile double biggestArea   = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) { }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // 1. RGB → HSV.
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            // 2. Threshold.
            Core.inRange(hsv, LOWER_HSV, UPPER_HSV, mask);

            // 3. Erode then dilate.
            if (ERODE_SIZE > 0) {
                Imgproc.erode(mask, mask, Imgproc.getStructuringElement(
                        Imgproc.MORPH_RECT, new Size(ERODE_SIZE, ERODE_SIZE)));
            }
            if (DILATE_SIZE > 0) {
                Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(
                        Imgproc.MORPH_RECT, new Size(DILATE_SIZE, DILATE_SIZE)));
            }

            // 4. Find contours.
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Mat maskCopy  = mask.clone();
            Imgproc.findContours(maskCopy, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            maskCopy.release();
            hierarchy.release();

            // 5. Filter by size and circularity.
            List<MatOfPoint> kept      = new ArrayList<>();
            List<Double>     keptAreas = new ArrayList<>();
            List<Double>     keptCircs = new ArrayList<>();
            double biggest = 0;
            int rejected = 0;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                // Too small → erase.
                if (area < MIN_BLOB_AREA) {
                    Imgproc.drawContours(mask, Collections.singletonList(contour),
                            -1, new Scalar(0), -1);
                    contour.release();
                    continue;
                }

                // Circularity: 4π × area ÷ perimeter².
                MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
                double perimeter = Imgproc.arcLength(c2f, true);
                c2f.release();
                double circ = (perimeter > 0)
                        ? 4 * Math.PI * area / (perimeter * perimeter) : 0;

                // Not round enough → erase from mask, but paint RED on frame later.
                if (circ < MIN_CIRCULARITY) {
                    Imgproc.drawContours(mask, Collections.singletonList(contour),
                            -1, new Scalar(0), -1);
                    rejected++;
                    // Still keep reference so we can paint it red below.
                    kept.add(contour);
                    keptAreas.add(area);
                    keptCircs.add(-circ);   // negative = rejected marker
                    continue;
                }

                kept.add(contour);
                keptAreas.add(area);
                keptCircs.add(circ);
                biggest = Math.max(biggest, area);
            }

            keptBlobs     = kept.size() - rejected;
            rejectedBlobs = rejected;
            biggestArea   = biggest;

            // 6. Show the cleaned mask.
            Imgproc.cvtColor(mask, frame, Imgproc.COLOR_GRAY2RGB);

            // 7. Paint rejected blobs red, and label everything.
            for (int i = 0; i < kept.size(); i++) {
                double circ = keptCircs.get(i);
                boolean isRejected = circ < 0;
                double displayCirc = Math.abs(circ);

                if (isRejected) {
                    Imgproc.drawContours(frame, Collections.singletonList(kept.get(i)),
                            -1, new Scalar(255, 0, 0), -1);   // red fill
                }

                Rect r = Imgproc.boundingRect(kept.get(i));
                // Line 1: pixel count
                Imgproc.putText(frame, String.format("%.0f px", keptAreas.get(i)),
                        new Point(r.x, Math.max(12, r.y - 16)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4,
                        new Scalar(0, 255, 0), 1);
                // Line 2: circularity
                Imgproc.putText(frame, String.format("c=%.2f", displayCirc),
                        new Point(r.x, Math.max(24, r.y - 2)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4,
                        new Scalar(0, 255, 255), 1);

                kept.get(i).release();
            }

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) { }
    }
}