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
//        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "intake"};
        String[] enabledComponents = {"logging", "gyroscope", "driving", "wobbleArm", "intake"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");
        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        robot.intake.turnOn();
        controlLoop();

        robot.intake.turnOn();
        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop(){
        while (opModeIsActive()){
            driveWithJoystick();
            driveWithDpad();

//            shooter();
            wobbleArm();
        }
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

        double averageWheelPower = (Math.abs(wpc.lf) + Math.abs(wpc.rf) + Math.abs(wpc.rb) + Math.abs(wpc.lb)) / 4;

        //smooth out the acceleration - f(x) = 0.6x^2 + 0.4x
        double scalerVal = 0.6*Math.pow(averageWheelPower, 2) + 0.4*averageWheelPower;
        wpc = WheelPowerConfig.multiply(wpc, scalerVal);

        robot.driving.setWheelPowers(wpc);
        robot.logging.setLog("Average wheel power", averageWheelPower);
    }
    private void driveWithDpad(){
        enabledDriveControls = (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left) ? 1 : enabledDriveControls;
        if(enabledDriveControls != 1)
            return;

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