package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareOuttake {
//    private DcMotor Intake_Motor = null;
    public DcMotor outtake_Left = null;
    public DcMotor outtake_Right = null;

    public Servo left_Tilt = null;
    /*Constructor*/
    public HardwareOuttake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //map and setup mode of slide motors
        outtake_Left = hardwareMap.get(DcMotor.class, "outtake_Left");
        outtake_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_Left.setDirection(DcMotor.Direction.FORWARD); //TODO: reverse or not?????
        outtake_Left.setPower(0);

        outtake_Right = hardwareMap.get(DcMotor.class, "outtake_Right");
        outtake_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_Right.setDirection(DcMotor.Direction.REVERSE); //TODO: reverse or not?????
        outtake_Right.setPower(0);




    }


    public void start(){

    }


    public void stop () {

    }
    public void leftSliderSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtake_Left.setTargetPosition(DesiredSliderPosition);
        outtake_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_Left.setPower(SliderPower);
        //do nothing while the current position is still less than desired position (or close to it by 20)
//        while (Slider_Motor_Right.getCurrentPosition() < (rightDesiredSliderPosition-20) ||
//                (Slider_Motor_Left.getCurrentPosition() < (leftDesiredSliderPosition-20))){
//        }                   //do nothing.
//        //TODO: not sure if you need to set the power to 0 at the end.
//        Slider_Motor_Right.setPower(0);
//        Slider_Motor_Left.setPower(0);
    }
    public void rightSliderSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtake_Right.setTargetPosition(DesiredSliderPosition);
        outtake_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_Right.setPower(SliderPower);
        //do nothing while the current position is still less than desired position (or close to it by 20)
//        while (Slider_Motor_Right.getCurrentPosition() < (rightDesiredSliderPosition-20) ||
//                (Slider_Motor_Left.getCurrentPosition() < (leftDesiredSliderPosition-20))){
//        }                   //do nothing.
//        //TODO: not sure if you need to set the power to 0 at the end.
//        Slider_Motor_Right.setPower(0);
//        Slider_Motor_Left.setPower(0);
    }

    public void highBasketSlides(){
        leftSliderSetPositionPower(0,0);
        rightSliderSetPositionPower(0,0);

    }

}