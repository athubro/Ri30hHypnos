package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "Shooter PID Tuning")
public class PIDTuning extends LinearOpMode {

    // PIDF TUNING VALUES
    public static double P = 10.0;
    public static double I = 3.0;
    public static double D = 0.0;
    public static double F = 12.0;

    public static double TARGET_VELOCITY = 1700;

    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;

    private double previousP = -1;
    private double previousI = -1;
    private double previousD = -1;
    private double previousF = -1;

    @Override
    public void runOpMode() {

        // SEND TELEMETRY TO DRIVER STATION AND DASHBOARD
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");

        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter PID Tuning Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // APPLY PIDF WHEN VALUES CHANGE
            if (P != previousP ||
                    I != previousI ||
                    D != previousD ||
                    F != previousF) {

                applyPIDF();
            }

            // CONTROLS
            if (gamepad1.a) {
                rightShooter.setVelocity(TARGET_VELOCITY);
                leftShooter.setVelocity(TARGET_VELOCITY);
            }

            if (gamepad1.b) {
                rightShooter.setVelocity(0);
                leftShooter.setVelocity(0);
            }

            // TELEMETRY
            telemetry.addLine("===== SHOOTER PID TUNING =====");

            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("F", F);

            telemetry.addLine("-----------------------------");

            telemetry.addData("Target Velocity", TARGET_VELOCITY);

            telemetry.addData("Right Velocity", rightShooter.getVelocity());
            telemetry.addData("Left Velocity", leftShooter.getVelocity());

            telemetry.addData("Right Error",
                    TARGET_VELOCITY - rightShooter.getVelocity());

            telemetry.addData("Left Error",
                    TARGET_VELOCITY - leftShooter.getVelocity());

            telemetry.addLine("-----------------------------");
            telemetry.addData("A", "Spin");
            telemetry.addData("B", "Stop");

            telemetry.update();
        }

        rightShooter.setVelocity(0);
        leftShooter.setVelocity(0);
    }

    private void applyPIDF() {

        PIDFCoefficients pidf = new PIDFCoefficients(
                P, I, D, F
        );

        rightShooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        leftShooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        previousP = P;
        previousI = I;
        previousD = D;
        previousF = F;
    }
}