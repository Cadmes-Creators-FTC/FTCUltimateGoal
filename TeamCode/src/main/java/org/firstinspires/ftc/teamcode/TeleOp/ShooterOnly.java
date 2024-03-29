package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "shooterOnly", group = "TestingTeleOps")
public class ShooterOnly extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "shooter"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop() {
        while (opModeIsActive()){
            ringShooter();
        }
    }

    private void ringShooter(){
        if (gamepad2.a)
            robot.shooter.turnOn(0.6);
        else if(gamepad2.b)
            robot.shooter.turnOn(0.5);
        else if(gamepad2.x)
            robot.shooter.turnOn(0.4);
        else if(gamepad2.y)
            robot.shooter.turnOn(0.3);
        else
            robot.shooter.turnOff();
    }
}
