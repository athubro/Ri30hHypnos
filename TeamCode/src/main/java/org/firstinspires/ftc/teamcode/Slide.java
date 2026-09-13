package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {

    Telemetry telemetry;
    DcMotorEx Slide;

    double kP = 1.0;
    double kI = 0;
    double kD = 0;
    double kF = 0;


    public Slide(HardwareMap hardwareMap, Telemetry telemetry) {
        Slide = hardwareMap.get(DcMotorEx.class, "slide");

        Slide.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    int targetpos;

    public void changeTarget(boolean left, boolean right) {
        if(left){
            targetpos += 100;
        }

        if(right){
            targetpos -= 100;
        }

    }
    public void runPosition() {
        Slide.setTargetPosition(targetpos);

        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 3. Give it power. The control hub will automatically slow down
        // or reverse direction depending on the target vs current position.
        Slide.setPower(0.8);
    }

    public void reset(){
        Slide.setTargetPosition(0);

        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 3. Give it power. The control hub will automatically slow down
        // or reverse direction depending on the target vs current position.
        Slide.setPower(0.8);

    }

    public int getCurrentTarget() {
        return targetpos;

    }




}
