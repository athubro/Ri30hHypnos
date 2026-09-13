package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {

    Telemetry telemetry;
    DcMotorEx Slide;
    Servo wrist;
    Servo claw;

    boolean busy;

    double kP = 1.0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    double slideKP=10;
    public Slide(HardwareMap hardwareMap, Telemetry telemetry) {
        Slide = hardwareMap.get(DcMotorEx.class, "slide");
        wrist = hardwareMap.get(Servo.class, "SlideWrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setPositionPIDFCoefficients(slideKP);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide.setPower(0.5);
        Slide.setTargetPosition(Slide.getCurrentPosition());
        //Slide.setTargetPosition(0);



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
        Slide.setPower(0.8);

        Slide.setTargetPosition(targetpos);
        //Slide.setPower(0.5);
    }

    public void reset(){
        Slide.setTargetPosition(0);


        // 3. Give it power. The control hub will automatically slow down
        // or reverse direction depending on the target vs current position.
        Slide.setPower(0.8);

    }

    public int getCurrentTarget() {
        return targetpos;

    }

    public void moveIntoPos(){


        Slide.setPower(0.8);


        Slide.setTargetPosition(873);

        //Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(Slide.getCurrentPosition() - 873) > 50) {

        }
        wrist.setPosition(0.5);

    }

    public void backToTransfer() {
        claw.setPosition(0.4);
        // res
        Slide.setPower(0.8);
        // t pos of wrist in 0.9
    }

    public void slideDown(){
        Slide.setTargetPosition(0);
    }

    public void dropMovement(){
        //slide at 5600
        //wrist position at 0.2
        //claw pos at 0.5 to open

        Slide.setPower(0.8);
        Slide.setTargetPosition(2160);

        while (Math.abs(Slide.getCurrentPosition() - 2160) > 50) {

        }



        wrist.setPosition(0.2);

        //intermediate slide pos is 873

    }

    public void openClaw(){
        claw.setPosition((0.5));
    }

    public void closeClaw(){
        claw.setPosition(0.2);
    }

    public void moveBackReset(){


        Slide.setPower(0.8);


        Slide.setTargetPosition(873);

        //Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(Slide.getCurrentPosition() - 873) > 50) {

        }
        wrist.setPosition(0.9);


    }






}
