package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "shooterOnly", group = "TestingTeleOps")
public class ShooterOnlyTeleOp extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] disabledComponents = {"gyroscope", "driving"};
        robot = new MainRobot(hardwareMap, telemetry, disabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        while (opModeIsActive()){
            RingShooter();
        }

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }

    private void RingShooter(){
        if (gamepad2.a)
            robot.shooter.turnOn(1);
        else if(gamepad2.b)
            robot.shooter.turnOn(0.9);
        else if(gamepad2.x)
            robot.shooter.turnOn(0.8);
        else if(gamepad2.y)
            robot.shooter.turnOn(0.6);
        else
            robot.shooter.turnOf();
    }
}
