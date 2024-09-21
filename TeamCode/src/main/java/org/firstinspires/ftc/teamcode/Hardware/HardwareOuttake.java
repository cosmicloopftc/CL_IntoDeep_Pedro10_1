package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareOuttake {
//    private DcMotor Intake_Motor = null;
    public DcMotor Outtake_Left = null;
    public DcMotor Outtake_Right = null;
    /*Constructor*/
    public HardwareOuttake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //map and setup mode of slide motors
        Outtake_Left = hardwareMap.get(DcMotor.class, "Intake_Slides");
        Outtake_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake_Left.setDirection(DcMotor.Direction.FORWARD); //TODO: reverse or not?????
        Outtake_Left.setPower(0);

        Outtake_Right = hardwareMap.get(DcMotor.class, "Intake_Slides");
        Outtake_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Outtake_Right.setDirection(DcMotor.Direction.REVERSE); //TODO: reverse or not?????
        Outtake_Right.setPower(0);


    }


    public void start(){

    }


    public void stop () {

    }

//example:  for Intake motor
//public method (function) for Intake motor power--to be accessible from anywhere
//    public void setPower(double i) {
//        Intake_Motor.setPower(i);
//    }
}