package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfigs.MainRobotConfig;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;

@Autonomous(name="BlueBuildPlateLine", group="blue")
public class TestAutonomous extends LinearOpMode {

    private MainRobotConfig robot;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new MainRobotConfig(hardwareMap, telemetry);
        //wait for imu to calibrate
        robot.WaitForGyroCalibration();

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        //start autonomous
        AutonomousSequence();

        telemetry.addData("State", "Done");
        telemetry.update();
    }

    //autonomous sequence
    private void AutonomousSequence(){

    }
}
