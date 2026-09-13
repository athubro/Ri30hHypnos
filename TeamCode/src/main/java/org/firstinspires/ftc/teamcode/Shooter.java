package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private final DcMotorEx motor;
    private final Telemetry telemetry;

    // PIDF CONSTANTS
    private static final double kP = 10.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 12.0;

    // Target velocity in ticks/second
    private double targetVelocity = 0;

    // D-pad button states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        motor = hardwareMap.get(DcMotorEx.class, "shooter");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update(boolean dpadUp, boolean dpadDown) {

        // Increase target by 100 ticks/second
        if (dpadUp && !lastDpadUp) {
            targetVelocity += 100;
        }

        // Decrease target by 100 ticks/second
        if (dpadDown && !lastDpadDown) {
            targetVelocity -= 100;
        }

        // Don't allow negative velocity
        if (targetVelocity < 0) {
            targetVelocity = 0;
        }

        // Save button states
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;

        // Set target velocity
        motor.setVelocity(targetVelocity);

        // Telemetry
        telemetry.addData("Shooter Target", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Shooter Velocity", "%.0f ticks/s", motor.getVelocity());
        telemetry.addData(
                "Shooter Error",
                "%.0f ticks/s",
                targetVelocity - motor.getVelocity()
        );

        telemetry.addData("Shooter kP", kP);
        telemetry.addData("Shooter kI", kI);
        telemetry.addData("Shooter kD", kD);
        telemetry.addData("Shooter kF", kF);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    public void stop() {
        targetVelocity = 0;
        motor.setVelocity(0);
    }
}