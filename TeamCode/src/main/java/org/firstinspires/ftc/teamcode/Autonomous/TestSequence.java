package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="TestAutonomous")
public class TestSequence extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        robot.gyroscope.setCurrentAngle(0);
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();

        robot.logging.clearLogs();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");

    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        robot.driving.driveToPositionForwardOnly(new Vector2(0, 100), null, 0.5);

        robot.driving.driveToPositionForwardOnly(new Vector2(100, 100), -90.0, 0.5);

        robot.driving.driveToPositionForwardOnly(new Vector2(100, 100), null, 0.5);
        robot.driving.driveToPositionForwardOnly(new Vector2(100, 200), null, 0.5);
        robot.driving.driveToPositionForwardOnly(new Vector2(0, 200), null, 0.5);
        robot.driving.driveToPositionForwardOnly(new Vector2(0, 0), 0.0, 0.5);
    }
}
