package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="blue")
public class TestAutonomous extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new MainRobot(hardwareMap, telemetry);
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        //wait for imu to calibrate
        robot.gyroscope.WaitForGyroCalibration();

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        //start autonomous
        //AutonomousSequence();
        while (!isStopRequested()){
            AutonomousSequence();

            telemetry.addData("State", "Done");
            telemetry.update();
        }
    }

    //autonomous sequence
    private void AutonomousSequence() throws InterruptedException {

    }

}
