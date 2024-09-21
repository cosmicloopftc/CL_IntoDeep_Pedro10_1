package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareDrivetrain {
    public DcMotor RFront_Motor = null;
    public DcMotor LFront_Motor = null;
    public DcMotor RBack_Motor = null;
    public DcMotor LBack_Motor = null;


    double newForward = 0, newRight = 0, driveTheta = 0, r = 0 ;
    /*Constructor*/
    public HardwareDrivetrain() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //*Define and Initialize Motors (need to use reference to actual OpMode)
        RFront_Motor = hardwareMap.get(DcMotor.class, "RFront_Motor");
        LFront_Motor = hardwareMap.get(DcMotor.class, "LFront_Motor");
        RBack_Motor = hardwareMap.get(DcMotor.class, "RBack_Motor");
        LBack_Motor = hardwareMap.get(DcMotor.class, "LBack_Motor");
        //Set Zero Power Behavior
        RFront_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFront_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBack_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBack_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //*Reset motor encoder to 0. Use this for our odometry pods.
        RFront_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFront_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBack_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBack_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //*Set Motor Mode to run with using encoder and just go based on power requested.
        RFront_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFront_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBack_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBack_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //*Setup Motor Direction

        //RFront_Motor.setDirection(DcMotor.Direction.REVERSE);
        //RBack_Motor.setDirection(DcMotor.Direction.REVERSE);
        //LFront_Motor.setDirection(DcMotor.Direction.FORWARD);
        //LBack_Motor.setDirection(DcMotor.Direction.FORWARD);

        RFront_Motor.setDirection(DcMotor.Direction.REVERSE);
        RBack_Motor.setDirection(DcMotor.Direction.REVERSE);

        setMotorPower(0, 0, 0, 0);


    }


    public void start(){

    }


    public void stop () {
        setMotorPower(0,0,0,0);
    }

    //public method (function) for drivetrain motor power--to be accessible from anywhere
    public void setMotorPower(double RF, double LF, double RB, double LB) {
        RFront_Motor.setPower(RF);
        LFront_Motor.setPower(LF);
        RBack_Motor.setPower(RB);
        LBack_Motor.setPower(LB);
    }

    //simplify driving move, based on Learn Java for FTC p. 145-146, Oct 2023
    //botheading is used for fieldOriented (driver's perspective driving)
    public void drive(double forward, double right, double rotate, double power, double botHeading, String drivingOrientation) {

        if (drivingOrientation == "fieldOriented") {
            driveTheta = Math.atan2(forward, right);                         //convert to polar
            r = Math.hypot(forward, right);                                  //convert to polar
            driveTheta = AngleUnit.normalizeRadians(driveTheta - botHeading);       // rotate angle
            newForward = r * Math.sin(driveTheta);
            newRight = 1.1 * r * Math.cos(driveTheta);  //factor = 1.1; correct for strafe imperfection; convert back to cartesian
        }
        if (drivingOrientation == "robotOriented") {
            newForward = forward;
            newRight = right;
        }
        double denominator = Math.max(Math.abs(newForward) + Math.abs(newRight) + Math.abs(rotate), 1);
        double frontLeftPower =     power*(newForward + newRight + rotate)/denominator;
        double frontRightPower = power*(newForward - newRight - rotate)/denominator;
        double backLeftPower = power*(newForward - newRight + rotate)/denominator;
        double backRightPower = power*(newForward + newRight - rotate)/denominator;

        setMotorPower(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
    }
}