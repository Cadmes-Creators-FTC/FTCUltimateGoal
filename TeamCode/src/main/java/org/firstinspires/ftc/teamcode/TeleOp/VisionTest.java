package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "visionTest", group = "TestingTeleOps")
public class VisionTest extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        while (opModeIsActive()){
            logOrangeVal();
        }

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }

    private void logOrangeVal(){
        robot.logging.setLog("RedVal", robot.ringStackHeightDetection.getRedVal());
        robot.logging.setLog("StackSize", robot.ringStackHeightDetection.getStackSize());
    }
}
