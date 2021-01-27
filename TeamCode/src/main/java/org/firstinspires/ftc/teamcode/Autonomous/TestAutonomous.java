package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="TestAutonomous")
public class TestAutonomous extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        robot.driving.driveToPosition(new Vector2(100, 100), 0.5);
        robot.driving.driveToPosition(new Vector2(0, 100), 0.5);
        robot.driving.driveToPosition(new Vector2(100, 0), 0.5);
        robot.driving.driveToPosition(new Vector2(0, 0), 0.5);
    }
}
