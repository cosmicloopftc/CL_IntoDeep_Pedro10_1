package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Runtime.getRuntime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.google.blocks.ftcrobotcontroller.util.AvailableTtsLocalesProvider;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareDrivetrain;
import org.firstinspires.ftc.teamcode.Hardware.HardwareRobot;

import java.util.List;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config    //need this to allow appearance in FtcDashboard Configuration to make adjust of variables
@TeleOp(group="Primary", name= "TeleOpV1")
public class TeleOpV1 extends OpMode {
    enum State{
        START,
    }
    private Follower follower;
    State state = State.START;
    HardwareRobot robot = new HardwareRobot();
    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();
    double botHeading;
    String drivingOrientation = "robotOriented";                //TODO: as default for Eduardo, but will also reset in init as well.
    double lastTime;

    public void init() {
        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);   //note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        robot.imu.resetYaw();      //reset the IMU/Gyro angle with each match.
        runtime.reset();

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AvailableTtsLocalesProvider FtcDashboard;

        //Encoders odometry = new Encoders();
        //Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        //ist<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime() - lastTime);
        telemetry.addData("lastTime", lastTime);




        //12-4-2023: Slide fine adjustment based on JAVA FOR FTC book
        switch (state) {
            case START:

                break;
        }

        //Drivetrain Movement:
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
    }

    public void init_loop() {

    }

    public void start() {
        robot.start();
        runtime.reset();
        drivingOrientation = "robotOriented";

        state = State.START;

    }
    @Override
    public void stop() {
        robot.stop();
    }
}
