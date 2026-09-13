package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class ColorVision {

    private OpenCvCamera camera;

    // Detection status
    private boolean detected = false;

    // Detected ball position
    private double centerX = 0;
    private double centerY = 0;

    // Detected ball size
    private double width = 0;
    private double height = 0;

    public ColorVision(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        WebcamName webcamName = hardwareMap.get(
                WebcamName.class,
                "Webcam 1"
        );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        webcamName,
                        cameraMonitorViewId
                );

        camera.setPipeline(new ElementPipeline());

        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {

                    @Override
                    public void onOpened() {

                        camera.startStreaming(
                                640,
                                480,
                                OpenCvCameraRotation.UPRIGHT
                        );
                    }

                    @Override
                    public void onError(int errorCode) {
                    }
                }
        );
    }

    // =========================
    // GETTERS
    // =========================

    public boolean isDetected() {
        return detected;
    }

    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    // =========================
    // OFFSET ANGLE
    // =========================

    public double getOffsetAngle() {

        // Center of the 640x480 image
        double imageCenterX = 320.0;

        // Approximate horizontal field of view
        double horizontalFOV = 60.0;

        // How many pixels the ball is from the center
        double pixelOffset = centerX - imageCenterX;

        // Convert pixel offset to angle
        return (pixelOffset / 640.0) * horizontalFOV;
    }

    // =========================
    // STOP CAMERA
    // =========================

    public void stop() {

        if (camera != null) {

            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    // =========================
    // PIPELINE
    // =========================

    private class ElementPipeline extends OpenCvPipeline {

        Mat hsv = new Mat();
        Mat mask = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            // Convert camera image to HSV
            Imgproc.cvtColor(
                    input,
                    hsv,
                    Imgproc.COLOR_RGB2HSV
            );

            // =========================
            // YELLOW COLOR RANGE
            // =========================

            // Lower bounds to cut out dark shadows, grey mats, and white walls
            Scalar lowerYellow = new Scalar(
                    18,   // Hue: Cuts out orange / red field perimeter elements
                    100,  // Saturation: STRICTLY cuts out white walls, white papers, and silver metal
                    60    // Value: STRICTLY cuts out pitch-black shadows and the dark gray foam tiles
            );

            // Upper bounds to cut out bright arena lights and green elements
            Scalar upperYellow = new Scalar(
                    32,   // Hue: Strict limit to avoid picking up the green field tape or green elements
                    255,  // Max Saturation
                    255   // Max Value
            );


            // Create binary mask
            Core.inRange(
                    hsv,
                    lowerYellow,
                    upperYellow,
                    mask
            );

            // =========================
            // FIND CONTOURS
            // =========================

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(
                    mask,
                    contours,
                    new Mat(),
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            // Reset detection every frame
            detected = false;

            centerX = 0;
            centerY = 0;
            width = 0;
            height = 0;

            // =========================
            // FIND LARGEST YELLOW OBJECT
            // =========================

            double largestArea = 0;

            Rect bestRect = null;

            for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);

                if (area > largestArea) {

                    largestArea = area;

                    bestRect = Imgproc.boundingRect(
                            contour
                    );
                }
            }

            // =========================
            // VALID DETECTION
            // =========================

            if (bestRect != null && largestArea > 500) {

                detected = true;

                centerX =
                        bestRect.x +
                                bestRect.width / 2.0;

                centerY =
                        bestRect.y +
                                bestRect.height / 2.0;

                width = bestRect.width;
                height = bestRect.height;

                // Draw bounding box
                Imgproc.rectangle(
                        input,
                        bestRect,
                        new Scalar(0, 255, 0),
                        3
                );

                // Draw center point
                Imgproc.circle(
                        input,
                        new Point(
                                centerX,
                                centerY
                        ),
                        5,
                        new Scalar(0, 255, 0),
                        -1
                );

                // Draw image center line
                Imgproc.line(
                        input,
                        new Point(320, 0),
                        new Point(320, 480),
                        new Scalar(255, 0, 0),
                        2
                );
            }

            return input;
        }
    }
}