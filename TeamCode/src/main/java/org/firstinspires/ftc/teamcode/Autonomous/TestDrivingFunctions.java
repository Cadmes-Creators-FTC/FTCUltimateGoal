package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestDrivingFunctions", group="TestAutonomous")
public class TestDrivingFunctions extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.resetWheelTicks();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        robot.gyroscope.setCurrentAngle(0);
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();
        while(!isStopRequested()){

        }

//        robot.logging.clearLogs();

//        robot.stopRobot();
//        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
//        robot.driving.driveToPosition(new Vector2(0, 100), 0.0, 1);

//        robot.driving.driveToPosition(new Vector2(100, 0), 0.0, 1);

//        robot.driving.driveToPosition(new Vector2(100, 100), 0.0, 1);

//        robot.driving.rotateToAngle(90, 1);

//        robot.driving.driveToPosition(new Vector2(0, 100), 90.0, 1);

        robot.driving.driveToPosition(new Vector2(100,100), 180.0, 1);
//        robot.driving.driveToPosition(new Vector2(0, 100), 90.0, 1);
    }
}
