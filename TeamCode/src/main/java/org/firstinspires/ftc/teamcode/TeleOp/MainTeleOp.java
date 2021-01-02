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
        String[] disabledComponents = {};
        robot = new MainRobot(hardwareMap, telemetry, disabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        while (opModeIsActive()){
            DriveWithController();
//            RingShooter();

                button();
            driveWithJoystick();
            ringShooter();
//            if (gamepad1.dpad_up)
//                button();
        }

        robot.isRunning = false;

        robot.logging.removeLog("power");
        robot.logging.setLog("state", "Stopped");
    }

    private void DriveWithController(){
        if (gamepad1.dpad_up||gamepad1.dpad_down)
            return;
        
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

        robot.logging.setLog("power", (wpc.lf+wpc.rf+wpc.rb+wpc.lb)/4);
    }

    private void ringShooter(){
        if (gamepad2.a)
            robot.shooter.turnOn(0.85);
        else
            robot.shooter.turnOf();
    }

    private void button(){
        double Test0 = 0;
        double Test1 = 0;
        double Test2 = 0;
        double Test3 = 0;
        double Counter = 0.1;
       WheelPowerConfig wpc = new WheelPowerConfig(
                0,
                0,
                0,
                0
       );
       //0 wpc.clamp();

        if (gamepad1.dpad_up) {
            wpc.lf = 0.5;
            wpc.lb = 0.5;
            wpc.rf = 0.5;
            wpc.rb = 0.5;
            robot.driving.setWheelPowers(wpc);
        }

        if (gamepad1.dpad_down) {
            wpc.lf = -0.5;
            wpc.lb = -0.5;
            wpc.rf = -0.5;
            wpc.rb = -0.5;
            robot.driving.setWheelPowers(wpc);
        }

        if (gamepad1.dpad_left) {
            wpc.lf = -0.5;
            wpc.lb = 0.5;
            wpc.rf = 0.5;
            wpc.rb = -0.5;
            robot.driving.setWheelPowers(wpc);
        }

        if (gamepad1.dpad_right){
            wpc.lf = 0.5;
            wpc.lb = -0.5;
            wpc.rf = -0.5;
            wpc.rb = 0.5;
            robot.driving.setWheelPowers(wpc);
        }
    }
}
