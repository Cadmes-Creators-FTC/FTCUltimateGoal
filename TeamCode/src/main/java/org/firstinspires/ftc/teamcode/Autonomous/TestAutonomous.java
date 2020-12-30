package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="blue")
public class TestAutonomous extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] disabledComponents = {};
        robot = new MainRobot(hardwareMap, telemetry, disabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        //start autonomous
        //AutonomousSequence();
        while (!isStopRequested()){
            autonomousSequence();

        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        robot.driving.driveToPosition(new Vector2(0, 100));
    }
}
