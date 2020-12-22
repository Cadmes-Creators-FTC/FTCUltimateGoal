package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "shooter_only", group = "TeleOp")
public class ShooterOnlyTeleOp extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new MainRobot(hardwareMap, telemetry);

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            RingShooter();
        }

        robot.isRunning = false;
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
