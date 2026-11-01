package org.firstinspires.ftc.teamcode.blobcv;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "Yellow Ball Detector (Logitech)", group = "Vision")
public class ColorSampler extends LinearOpMode {

    // Custom overlay processor to draw yellow fill mapping inside the detected ball
    static class YellowBallOverlayProcessor implements VisionProcessor {
        private final ColorBlobLocatorProcessor blobLocator;
        private final Paint yellowFillPaint;
        private final Paint yellowBorderPaint;

        public YellowBallOverlayProcessor(ColorBlobLocatorProcessor blobLocator) {
            this.blobLocator = blobLocator;

            // Semi-transparent yellow fill mapping the inside of the ball
            yellowFillPaint = new Paint();
            yellowFillPaint.setColor(Color.YELLOW);
            yellowFillPaint.setStyle(Paint.Style.FILL);
            yellowFillPaint.setAlpha(140); // Translucent yellow overlay inside

            // Solid yellow outline for the edge
            yellowBorderPaint = new Paint();
            yellowBorderPaint.setColor(Color.YELLOW);
            yellowBorderPaint.setStyle(Paint.Style.STROKE);
            yellowBorderPaint.setStrokeWidth(5);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            blobLocator.init(width, height, calibration);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return blobLocator.processFrame(frame, captureTimeNanos);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            blobLocator.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);

            // Draw filled yellow overlay mapping inside the detected balls
            List<ColorBlobLocatorProcessor.Blob> blobs = blobLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    100, 20000, blobs);

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                Circle circle = b.getCircle();
                float cx = circle.getX() * scaleBmpPxToCanvasPx;
                float cy = circle.getY() * scaleBmpPxToCanvasPx;
                float radius = circle.getRadius() * scaleBmpPxToCanvasPx;

                // Fill inside of ball with yellow
                canvas.drawCircle(cx, cy, radius, yellowFillPaint);
                // Draw crisp border around edge
                canvas.drawCircle(cx, cy, radius, yellowBorderPaint);
            }
        }
    }

    @Override
    public void runOpMode() {

        // Build official FTC ColorBlobLocatorProcessor for YELLOW objects
        ColorBlobLocatorProcessor yellowBallLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(0) // Custom overlay processor handles circle drawing and fill
                .setBlurSize(5)
                .setDilateSize(10)
                .setErodeSize(10)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        YellowBallOverlayProcessor overlayProcessor = new YellowBallOverlayProcessor(yellowBallLocator);

        // Build VisionPortal for Logitech camera ("Webcam 1")
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(overlayProcessor)
                .build();

        telemetry.addLine("Logitech Camera Active! Place a yellow ball in front of camera.");
        telemetry.update();

        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("=== LOGITECH CAMERA YELLOW BALL DETECTOR ===");

            // Retrieve detected yellow blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = yellowBallLocator.getBlobs();

            // Filter out small noise
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    100, 20000, blobs);

            if (!blobs.isEmpty()) {
                telemetry.addLine("🟢 YELLOW BALL DETECTED!");
                telemetry.addData("Yellow Blobs Count", blobs.size());

                // Report largest detected ball details
                ColorBlobLocatorProcessor.Blob mainBlob = blobs.get(0);
                Circle circle = mainBlob.getCircle();
                telemetry.addData("Ball Center (X, Y)", (int) circle.getX() + ", " + (int) circle.getY());
                telemetry.addData("Ball Radius", (int) circle.getRadius() + " px");
                telemetry.addData("Contour Area", mainBlob.getContourArea() + " px²");
            } else {
                telemetry.addLine("⚪ NO YELLOW BALL DETECTED");
            }

            telemetry.update();
            sleep(30);
        }

        portal.close();
    }
}