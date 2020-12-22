package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;

@TeleOp(name = "Main_Robot", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new MainRobot(hardwareMap, telemetry);
        //wait for imu to calibrate
        robot.gyroscope.waitForGyroCalibration();

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            driveWithJoystick();
//            ringShooter();
            if (gamepad1.dpad_up)
                button();
        }

        robot.isRunning = false;
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
        robot.driving.setWheelPowers(wpc);
    }

    private void ringShooter(){
        if (gamepad2.a)
            robot.shooter.turnOn(1);
        else
            robot.shooter.turnOf();
    }

    private void button(){
        boolean dPad = gamepad1.dpad_up;
        double Test0 = 0;
        double Test1 = 0;
        double Test2 = 0;
        double Test3 = 0;
        double Counter = 0.1;

        WheelPowerConfig wpc = new WheelPowerConfig(
                Test0 + Counter,
                Test1 + Counter,
                Test2 + Counter,
                Test3 + Counter
        );
        wpc.clamp();
        
        if (dPad)
            wpc.lf = Counter + Math.pow(wpc.lf, 1);
            wpc.lb = Counter + Math.pow(wpc.lb, 1);
            wpc.rf = Counter + Math.pow(wpc.rf, 1);
            wpc.rb = Counter + Math.pow(wpc.rb, 1);


    }
}
