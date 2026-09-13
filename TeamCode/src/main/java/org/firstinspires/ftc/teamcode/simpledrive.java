package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Tank Drive")
public class simpledrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        TankDrive drive;
        Intake intake;
        Shooter shooter;
        ColorVision vision;
        Slide slide;
        boolean trans;

        drive = new TankDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        intake = new Intake(
                hardwareMap,
                telemetry
        );

        shooter = new Shooter(
                hardwareMap,
                telemetry
        );

        slide = new Slide(hardwareMap, telemetry);

        vision = new ColorVision(hardwareMap);

        // =========================
        // AUTO PICKUP VARIABLES
        // =========================

        boolean autoPickup = false;
        boolean lastA = false;

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // VISION
            // =========================

            boolean detected = vision.isDetected();

            double offsetAngle = vision.getOffsetAngle();

            // =========================
            // A BUTTON
            // =========================

            if (gamepad1.a && !lastA) {

                // Start auto pickup
                if (!autoPickup) {
                    autoPickup = true;
                }

                // Stop auto pickup
                else {
                    autoPickup = false;
                }
            }

            if (gamepad2.x) {
                shooter.motor.setVelocity(200);

                slide.moveIntoPos();
                sleep(1000);
                slide.backToTransfer();
                sleep(1000);
                slide.slideDown();
                sleep(1000);
                slide.closeClaw();

            }

            if (gamepad2.y) {
                slide.dropMovement();
                sleep(1000);
                slide.openClaw();

            }

            if (gamepad2.b){

                slide.closeClaw();
                sleep(300);
                slide.moveBackReset();
                sleep(1000);
                slide.slideDown();

            }



            lastA = gamepad1.a;

            slide.changeTarget(gamepad2.dpadLeftWasPressed(), gamepad2.dpadRightWasPressed());

            if (gamepad2.b) {
                slide.runPosition();
            }




            // =========================
            // AUTO PICKUP
            // =========================

            if (autoPickup) {

                if (detected) {

                    /*
                     * Turn toward the ball.
                     *
                     * Positive offset = ball is to the right
                     * Negative offset = ball is to the left
                     */

                    double turnPower = offsetAngle * 0.025;

                    // Limit turning power
                    if (turnPower > 0.5) {
                        turnPower = 0.5;
                    }

                    if (turnPower < -0.5) {
                        turnPower = -0.5;
                    }

                    /*
                     * If the ball is far from center,
                     * turn toward it.
                     *
                     * If it is close to center,
                     * drive forward.
                     */

                    if (Math.abs(offsetAngle) > 5.0) {

                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(0, 0),
                                        turnPower
                                )
                        );

                    } else {

                        // Drive toward ball
                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(0.5, 0),
                                        0
                                )
                        );
                    }

                    // Run intake while picking up
                    intake.intakeMotor.setPower(1.0);
                    intake.left.setPower(-1.0);
                    intake.right.setPower(1.0);

                } else {

                    // Ball disappeared
                    autoPickup = false;

                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(0, 0),
                                    0
                            )
                    );

                    intake.intakeMotor.setPower(0);
                    intake.left.setPower(0);
                    intake.right.setPower(0);
                }

            } else {

                // =========================
                // NORMAL DRIVE
                // =========================

                double forwardInput =
                        -gamepad1.left_stick_y;

                double rotationInput =
                        -gamepad1.right_stick_x;

                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        forwardInput,
                                        0
                                ),
                                rotationInput
                        )
                );

                // =========================
                // NORMAL INTAKE
                // =========================

                double intakeInput =
                        gamepad1.left_trigger;

                intake.intakeMotor.setPower(
                        intakeInput
                );

                trans = gamepad1.left_bumper;
                intake.left.setPower(-intakeInput);
                intake.right.setPower(intakeInput);
                if (trans){
                    intake.transfer.setPower(-1);
                }else{
                    intake.transfer.setPower(0);
                }

            }

            // =========================
            // SHOOTER
            // =========================

            if (gamepad1.right_trigger > 0.1) {
                shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);

            } else {
                shooter.stop();
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addLine("===== VISION =====");

            telemetry.addData(
                    "Yellow Detected",
                    detected
            );

            telemetry.addData(
                    "Ball X",
                    "%.1f",
                    vision.getCenterX()
            );

            telemetry.addData(
                    "Ball Y",
                    "%.1f",
                    vision.getCenterY()
            );

            telemetry.addData(
                    "Offset Angle",
                    "%.2f degrees",
                    offsetAngle
            );

            telemetry.addData(
                    "Auto Pickup",
                    autoPickup
            );

            telemetry.addLine("===== DRIVE =====");

            telemetry.addData(
                    "Forward",
                    "%.2f",
                    -gamepad1.left_stick_y
            );

            telemetry.addData(
                    "Rotation",
                    "%.2f",
                    -gamepad1.right_stick_x
            );

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData(
                    "Target Velocity",
                    "%.0f ticks/s",
                    shooter.getTargetVelocity()
            );

            telemetry.addData(
                    "Slide Current Target",
                    slide.getCurrentTarget()
            );
            telemetry.addData(
                    "Slide Current Position",
                    slide.Slide.getCurrentPosition()
            );

            telemetry.addData(
                    "Current Velocity",
                    "%.0f ticks/s",
                    shooter.getCurrentVelocity()
            );

            telemetry.addData(
                    "Velocity Error",
                    "%.0f ticks/s",
                    shooter.getTargetVelocity()
                            - shooter.getCurrentVelocity()
            );

            telemetry.addData(
                    "slide power",
                    "%.0f ",
                    slide.Slide.getPower()
            );

            telemetry.update();
        }

        // =========================
        // STOP EVERYTHING
        // =========================

        shooter.stop();

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );

        intake.intakeMotor.setPower(0);
        intake.left.setPower(0);
        intake.right.setPower(0);

        vision.stop();
    }
}