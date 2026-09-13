package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Tank Drive")
public class simpledrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        TankDrive drive;
        Intake intake;
        Shooter shooter;


        drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);




        waitForStart();

        while (opModeIsActive()) {

            double forwardInput = -gamepad1.left_stick_y;
            double rotationInput = -gamepad1.right_stick_x;
            double intakeInput = gamepad1.left_trigger;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forwardInput, 0),
                    rotationInput
            ));

            intake.intakeMotor.setPower(intakeInput);
            intake.left.setPower(-1.0);
            intake.right.setPower(1.0);

            shooter.update(gamepad1.dpad_up, gamepad1.dpad_down);



        }
    }
}