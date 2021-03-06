package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestRingValues", group="TestAutonomous")
public class TestRingValues extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");

        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        while (robot.isRunning){
            robot.logging.setLog("redVal", robot.ringStackDetection.getRedVal());
            robot.logging.setLog("numOfRings", robot.ringStackDetection.getStackSize());
        }
    }
}
