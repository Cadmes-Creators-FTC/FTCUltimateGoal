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
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "intake", "conveyor"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");
        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop(){
        while (opModeIsActive()){
            driveWithJoystick();
            slowMovement();
            setDrivingDirection();

            intake();
            conveyor();
            shooter();

            wobbleArm();
        }
    }


    boolean intakeStateChanged = false;
    private void intake(){
        if (gamepad2.b && !intakeStateChanged){
            intakeStateChanged = true;
            if (robot.intake.isOn())
                robot.intake.turnOff();
            else
                robot.intake.turnOn();
        }
        if(!gamepad2.b)
            intakeStateChanged = false;
    }

    boolean conveyorStateChanged = false;
    private void conveyor(){
        if (gamepad2.left_bumper && !conveyorStateChanged){
            conveyorStateChanged = true;
            if (robot.conveyor.isOn())
                robot.conveyor.turnOff();
            else
                robot.conveyor.turnOn(0.3);
        }
        if(!gamepad2.left_bumper)
            conveyorStateChanged = false;
    }

    boolean shooterStateChanged = false;
    private void shooter(){
        if (gamepad2.a && !shooterStateChanged){
            shooterStateChanged = true;
            if (robot.shooter.isOn())
                robot.shooter.turnOff();
            else
                robot.shooter.turnOn(1);
        }
        if(!gamepad2.a)
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
                robot.wobbleArm.openGripper();
        }
        if(!gamepad2.y)
            wobbleArmGripperStateChanged = false;
    }

    int enabledDriveControls = 0;
    private void driveWithJoystick(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        enabledDriveControls = (joyX != 0 || joyY != 0 || joyR != 0) ? 0 : enabledDriveControls;
        if(enabledDriveControls != 0)
            return;

        //smooth out the joysticks - f(x) = 0.6x^2 + 0.4x
        if(joyX != 0)
            joyX = 0.6*(Math.pow(joyX, 2)*(joyX/Math.abs(joyX)))+0.4*joyX;
        if(joyY != 0)
            joyY = 0.6*(Math.pow(joyY, 2)*(joyY/Math.abs(joyY)))+0.4*joyY;
        if(joyR != 0)
            joyR = 0.6*(Math.pow(joyR, 2)*(joyR/Math.abs(joyR)))+0.4*joyR;

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

        robot.driving.setWheelPowers(wpc);
    }
    private void slowMovement(){
        enabledDriveControls = (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.left_bumper || gamepad2.right_bumper) ? 1 : enabledDriveControls;
        if(enabledDriveControls != 1)
            return;

        double power = 0.2;

        if (gamepad1.dpad_up) {
            WheelPowerConfig wpc = new WheelPowerConfig(power, power, power, power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_down) {
            WheelPowerConfig wpc = new WheelPowerConfig(-power, -power, -power, -power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_left) {
            WheelPowerConfig wpc = new WheelPowerConfig(-1.5*power, 1.5*power, -1.5*power, 1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_right){
            WheelPowerConfig wpc = new WheelPowerConfig(1.5*power, -1.5*power, 1.5*power, -1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.left_bumper) {
            WheelPowerConfig wpc = new WheelPowerConfig(-1.5*power, 1.5*power, 1.5*power, -1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.right_bumper) {
            WheelPowerConfig wpc = new WheelPowerConfig(1.5*power, -1.5*power, -1.5*power, 1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else {
            WheelPowerConfig wpc = new WheelPowerConfig(0, 0, 0, 0);
            robot.driving.setWheelPowers(wpc);
        }
    }
    private void setDrivingDirection(){
        if(gamepad1.a)
            robot.driving.setReverseDriving(true);
        if(gamepad1.b)
            robot.driving.setReverseDriving(false);
    }
}