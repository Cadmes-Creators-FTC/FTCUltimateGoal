package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;

@TeleOp(name = "main", group = "GameTeleOps")
public class Main extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        while (opModeIsActive()){
            driveWithJoystick();
            driveWithDpad();

            ringShooter();
            wobbleArm();
        }

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }
        
    private void driveWithJoystick(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        //reverse y joystick
        joyY *= -1;

        //create wheel power config
        WheelPowerConfig wpc = new WheelPowerConfig(
                joyY + joyX + joyR,
                joyY - joyX - joyR,
                joyY + joyX - joyR,
                joyY - joyX + joyR
        );
        wpc.clamp();

        //smooth out the acceleration
        //f(x) = 0.6x^2 + 0.4x
        wpc.lf = 0.6*Math.pow(wpc.lf, 3) + 0.4*wpc.lf;
        wpc.rf = 0.6*Math.pow(wpc.rf, 3) + 0.4*wpc.rf;
        wpc.rb = 0.6*Math.pow(wpc.rb, 3) + 0.4*wpc.rb;
        wpc.lb = 0.6*Math.pow(wpc.lb, 3) + 0.4*wpc.lb;

        if(wpc.lf != 0 && wpc.rf != 0 && wpc.rb != 0 && wpc.lb != 0){
            robot.driving.setWheelPowers(wpc);
            robot.logging.setLog("Average wheel power", (wpc.lf+wpc.rf+wpc.rb+wpc.lb)/4);
        }
    }

    private void ringShooter(){
        if (gamepad2.a)
            robot.shooter.turnOn(0.85);
        else
            robot.shooter.turnOf();
    }

    private void driveWithDpad(){
        boolean dpadEnabled = false;
        WheelPowerConfig wpc = new WheelPowerConfig(0, 0, 0, 0);

        if (gamepad1.dpad_up) {
            dpadEnabled = true;
            wpc.lf = 0.5;
            wpc.lb = 0.5;
            wpc.rf = 0.5;
            wpc.rb = 0.5;
        }else if (gamepad1.dpad_down) {
            dpadEnabled = true;
            wpc.lf = -0.5;
            wpc.lb = -0.5;
            wpc.rf = -0.5;
            wpc.rb = -0.5;
        }else if (gamepad1.dpad_left) {
            dpadEnabled = true;
            wpc.lf = -0.5;
            wpc.lb = 0.5;
            wpc.rf = 0.5;
            wpc.rb = -0.5;
        }else if (gamepad1.dpad_right){
            dpadEnabled = true;
            wpc.lf = 0.5;
            wpc.lb = -0.5;
            wpc.rf = -0.5;
            wpc.rb = 0.5;
        }

        if(dpadEnabled)
            robot.driving.setWheelPowers(wpc);
    }

    private void wobbleArm(){
        if (gamepad2.left_bumper){
            robot.wobbleArm.armDown();
        } else if (gamepad2.right_bumper){
            robot.wobbleArm.armUp();
        }
    }
}