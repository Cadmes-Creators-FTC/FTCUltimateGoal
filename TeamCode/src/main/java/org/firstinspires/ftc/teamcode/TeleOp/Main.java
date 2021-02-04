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
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "intake"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        startComponents();
        controlLoop();
        stopComponents();

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }

    private void startComponents(){
        robot.intake.turnOn();
    }
    private void controlLoop(){
        while (opModeIsActive()){
            driveWithJoystick();
            driveWithDpad();

            shooter();
            wobbleArm();
        }
    }
    private void stopComponents(){
        robot.intake.turnOff();
    }


    boolean shooterStateChanged = false;
    private void shooter(){
        if (gamepad2.b && !shooterStateChanged){
            shooterStateChanged = true;
            if (robot.shooter.isOn())
                robot.shooter.turnOff();
            else
                robot.shooter.turnOn(0.85);
        }
        if(!gamepad2.b)
            shooterStateChanged = false;
    }

    boolean wobbleArmGripperStateChanged = false;
    private void wobbleArm(){
        if (gamepad2.right_trigger > 0){
            robot.wobbleArm.armDown();
        } else if (gamepad2.right_bumper){
            robot.wobbleArm.armUp();
        }

        if (gamepad2.y && !wobbleArmGripperStateChanged){
            wobbleArmGripperStateChanged = true;
            if (robot.wobbleArm.isGripperOpen())
                robot.wobbleArm.closeGripper();
            else
                robot.wobbleArm.closeGripper();
        }
        if(!gamepad2.y)
            wobbleArmGripperStateChanged = false;
    }

    private void driveWithJoystick(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        boolean driveWithJoystickEnabled = joyX != 0 || joyY != 0 || joyR != 0;
        if(!driveWithJoystickEnabled)
            return;;

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
        robot.logging.setLog("Average wheel power", (wpc.lf+wpc.rf+wpc.rb+wpc.lb)/4);
    }
    private void driveWithDpad(){
        if (gamepad1.dpad_up) {
            WheelPowerConfig wpc = new WheelPowerConfig(0.5, 0.5, 0.5, 0.5);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_down) {
            WheelPowerConfig wpc = new WheelPowerConfig(-0.5, -0.5, -0.5, -0.5);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_left) {
            WheelPowerConfig wpc = new WheelPowerConfig(-0.5, 0.5, -0.5, 0.5);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_right){
            WheelPowerConfig wpc = new WheelPowerConfig(0.5, -0.5, 0.5, -0.5);
            robot.driving.setWheelPowers(wpc);
        }
    }
}