package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//TODO: program the Servo position


public class HardwareIntake {

    public DcMotor intake_Slides = null;


    public CRServo intake_LeftWheel = null;
    public CRServo intake_RightWheel = null;
    public Servo intake_LeftTilt = null;
    public Servo intake_RightTilt = null;
    //Servo Test
//    public Servo Servo_Test = null;


    /*Constructor*/
    public HardwareIntake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map

        //map and setup mode of Intake Slide Motor
        intake_Slides = hardwareMap.get(DcMotor.class, "Intake_Slides");
        //intake slide motor behaviors
        intake_Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_Slides.setDirection(DcMotor.Direction.FORWARD); //TODO: reverse or not?????
        intake_Slides.setPower(0);

        //map and setup mode of Intake Continuous Servos
        intake_LeftWheel = hardwareMap.get(CRServo.class, "Intake_LeftWheel");
        intake_RightWheel = hardwareMap.get(CRServo.class, "Intake_RightWheel");
        intake_LeftTilt = hardwareMap.get(Servo.class, "Intake_LeftTilt");
        intake_RightTilt = hardwareMap.get(Servo.class, "Intake_RightTilt");



        //TEST SERVO
//        Servo_Test = hardwareMap.get(Servo.class, "Servo_Test");

    }


    public void start(){

    }


    public void stop () {

    }

    //TODO: Method for entire intake process
    //public method (function) for intaking sample
    public void IntakeIN () {
        intake_LeftWheel.setPower(1);
        intake_RightWheel.setPower(-1);
    }


    //public method (function) for spitting out sample
    public void IntakeOUT() {
        intake_LeftWheel.setPower(-1);
        intake_RightWheel.setPower(1);
    }

    //public method (function) for stopping the intake
    public void IntakeSTOP() {
        intake_LeftWheel.setPower(0);
        intake_RightWheel.setPower(0);
    }



}