package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorVision {

    private OpenCvWebcam webcam;
    private final YellowPipeline pipeline;

    private boolean autoPickup = false;

    private static final double TURN_MULTIPLIER = 0.025;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double FORWARD_POWER = 0.5;
    private static final double ALIGNMENT_THRESHOLD = 5.0;

    public ColorVision(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        // Correct webcam initialization.
        WebcamName cameraName = hardwareMap.get(
                WebcamName.class,
                "Webcam 1"
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                cameraName,
                cameraMonitorViewId
        );

        pipeline = new YellowPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {

                    @Override
                    public void onOpened() {
                        webcam.startStreaming(
                                640,
                                480,
                                OpenCvCameraRotation.UPRIGHT
                        );
                    }

                    @Override
                    public void onError(int errorCode) {
                        // Camera failed to open.
                    }
                }
        );
    }

    /**
     * Handles automatic pickup.
     *
     * @param togglePressed True when the toggle button is pressed.
     * @param drive The robot's drive subsystem.
     * @param intake The robot's intake subsystem.
     * @return True if automatic pickup is handling the robot.
     */
    public boolean updateAutoPickup(
            boolean togglePressed,
            TankDrive drive,
            Intake intake
    ) {

        // Toggle automatic pickup.
        if (togglePressed) {
            autoPickup = !autoPickup;

            // Stop the robot when automatic pickup is disabled.
            if (!autoPickup) {
                stopPickup(drive, intake);
                return true;
            }
        }

        // Automatic pickup is disabled.
        if (!autoPickup) {
            return false;
        }

        // Stop if the target disappears.
        if (!isDetected()) {
            autoPickup = false;
            stopPickup(drive, intake);
            return true;
        }

        // Calculate the target's horizontal angle.
        double offsetAngle = getOffsetAngle();

        // Convert the angle into turning power.
        double turnPower = offsetAngle * TURN_MULTIPLIER;

        // Limit the turning power.
        turnPower = Math.max(
                -MAX_TURN_POWER,
                Math.min(MAX_TURN_POWER, turnPower)
        );

        // Turn toward the target until aligned.
        if (Math.abs(offsetAngle) > ALIGNMENT_THRESHOLD) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            turnPower
                    )
            );

        } else {

            // Drive forward when aligned.
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(FORWARD_POWER, 0),
                            0
                    )
            );
        }

        // Run the intake while approaching the target.
        intake.intakeMotor.setPower(1.0);
        intake.left.setPower(-1.0);
        intake.right.setPower(1.0);

        return true;
    }

    /**
     * Stops the drive and intake.
     */
    private void stopPickup(TankDrive drive, Intake intake) {

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );

        intake.intakeMotor.setPower(0);
        intake.left.setPower(0);
        intake.right.setPower(0);
        intake.transfer.setPower(0);
    }

    /**
     * Returns whether automatic pickup is enabled.
     */
    public boolean isAutoPickupActive() {
        return autoPickup;
    }

    /**
     * Returns whether a yellow object is detected.
     */
    public boolean isDetected() {
        return pipeline.isDetected();
    }

    /**
     * Returns the detected object's center X coordinate.
     */
    public int getCenterX() {
        return pipeline.getCenterX();
    }

    /**
     * Returns the detected object's center Y coordinate.
     */
    public int getCenterY() {
        return pipeline.getCenterY();
    }

    /**
     * Returns the detected object's width.
     */
    public int getWidth() {
        return pipeline.getWidth();
    }

    /**
     * Returns the detected object's height.
     */
    public int getHeight() {
        return pipeline.getHeight();
    }

    /**
     * Estimates the horizontal angle between the target
     * and the center of the camera image.
     *
     * Assumes a 60-degree horizontal field of view.
     */
    public double getOffsetAngle() {

        if (!isDetected()) {
            return 0;
        }

        return (getCenterX() - 320) / 640.0 * 60.0;
    }

    /**
     * Stops the camera and disables automatic pickup.
     */
    public void stop() {

        autoPickup = false;

        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    /**
     * OpenCV pipeline for detecting yellow objects.
     */
    private static class YellowPipeline extends OpenCvPipeline {

        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();

        private final List<MatOfPoint> contours = new ArrayList<>();

        private volatile boolean detected = false;

        private volatile int centerX = 0;
        private volatile int centerY = 0;
        private volatile int width = 0;
        private volatile int height = 0;

        @Override
        public Mat processFrame(Mat input) {

            // Convert the camera image to HSV.
            Imgproc.cvtColor(
                    input,
                    hsv,
                    Imgproc.COLOR_RGB2HSV
            );

            // Threshold for yellow.
            Core.inRange(
                    hsv,
                    new Scalar(18, 100, 60),
                    new Scalar(32, 255, 255),
                    mask
            );

            // Find yellow contours.
            contours.clear();

            Imgproc.findContours(
                    mask,
                    contours,
                    hierarchy,
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            // Find the largest contour.
            double largestArea = 0;
            Rect largestRect = null;

            for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);

                if (area > largestArea) {
                    largestArea = area;
                    largestRect = Imgproc.boundingRect(contour);
                }

                contour.release();
            }

            // Accept objects with an area greater than 500 pixels.
            if (largestRect != null && largestArea > 500) {

                detected = true;

                centerX = largestRect.x + largestRect.width / 2;
                centerY = largestRect.y + largestRect.height / 2;

                width = largestRect.width;
                height = largestRect.height;

                // Draw the bounding box.
                Imgproc.rectangle(
                        input,
                        largestRect,
                        new Scalar(0, 255, 0),
                        2
                );

                // Draw the center of the detected object.
                Imgproc.circle(
                        input,
                        new Point(centerX, centerY),
                        5,
                        new Scalar(255, 0, 0),
                        -1
                );

            } else {

                detected = false;

                centerX = 0;
                centerY = 0;
                width = 0;
                height = 0;
            }

            // Draw a vertical line through the center of the image.
            Imgproc.line(
                    input,
                    new Point(320, 0),
                    new Point(320, 480),
                    new Scalar(255, 255, 255),
                    1
            );

            return input;
        }

        public boolean isDetected() {
            return detected;
        }

        public int getCenterX() {
            return centerX;
        }

        public int getCenterY() {
            return centerY;
        }

        public int getWidth() {
            return width;
        }

        public int getHeight() {
            return height;
        }
    }
}