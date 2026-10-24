package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Simple Drive", group = "TeleOp")
public class simpledrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize drivetrain.
        TankDrive drive = new TankDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        // Initialize intake.
        Intake intake = new Intake(hardwareMap, telemetry);

        // Initialize shooter and Limelight auto-aim.
        Shooter shooter = new Shooter(
                hardwareMap,
                telemetry,
                drive
        );

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Gamepad 1: Drive and shooter");
        telemetry.addLine("Gamepad 2 A: Toggle AprilTag auto-aim");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ------------------------------------------
            // AUTO-AIM: GAMEPAD 2 A
            // ------------------------------------------

            boolean autoAimHandlesDrive = shooter.updateAutoAim(
                    gamepad2.aWasPressed()
            );

            // ------------------------------------------
            // MANUAL DRIVE
            // ------------------------------------------

            if (!autoAimHandlesDrive) {

                double forwardInput = -gamepad1.left_stick_y;
                double rotationInput = -gamepad1.right_stick_x;

                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(forwardInput, 0),
                                rotationInput
                        )
                );
            }

            // ------------------------------------------
            // INTAKE
            // ------------------------------------------

            // Keep manual intake control available.
            double intakePower = gamepad1.left_trigger;

            intake.intakeMotor.setPower(intakePower);
            intake.left.setPower(-intakePower);
            intake.right.setPower(intakePower);

            if (gamepad1.left_bumper) {
                intake.transfer.setPower(-1.0);
            } else {
                intake.transfer.setPower(0);
            }

            // ------------------------------------------
            // SHOOTER
            // ------------------------------------------

            if (gamepad1.right_trigger > 0.1) {

                shooter.update(
                        gamepad1.dpad_up,
                        gamepad1.dpad_down
                );

            } else {
                shooter.stop();
            }

            // ------------------------------------------
            // TELEMETRY
            // ------------------------------------------

            telemetry.addData(
                    "Auto Aim",
                    shooter.isAutoAimEnabled() ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Selected AprilTag",
                    shooter.getSelectedTagId()
            );

            telemetry.addData(
                    "Tag Distance",
                    "%.2f m",
                    shooter.getSelectedTagDistance()
            );

            telemetry.addData(
                    "Tag Angle",
                    "%.2f degrees",
                    shooter.getSelectedTagAngle()
            );

            telemetry.addData(
                    "Flywheel 1 Velocity",
                    "%.1f",
                    shooter.getFlywheel1Velocity()
            );

            telemetry.addData(
                    "Flywheel 2 Velocity",
                    "%.1f",
                    shooter.getFlywheel2Velocity()
            );

            telemetry.addData(
                    "Average Flywheel Velocity",
                    "%.1f",
                    shooter.getCurrentVelocity()
            );

            telemetry.addData(
                    "Target Velocity",
                    "%.1f",
                    shooter.getTargetVelocity()
            );

            telemetry.addData(
                    "Shooter Up To Speed",
                    shooter.upToSpeed
            );

            telemetry.update();
        }

        // ------------------------------------------
        // SHUTDOWN
        // ------------------------------------------

        shooter.shutdown();

        intake.intakeMotor.setPower(0);
        intake.left.setPower(0);
        intake.right.setPower(0);
        intake.transfer.setPower(0);
    }
}