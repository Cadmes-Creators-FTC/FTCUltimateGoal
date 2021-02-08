package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "wheelsOnly", group = "TestingTeleOps")
public class WheelsOnly extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "driving"};
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
    public void controlLoop(){
        while (opModeIsActive()){
            driveWithDpad();
            driveWithJoystick();
        }
    }

    int enabledDriveControls = 0;
    private void driveWithJoystick(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        enabledDriveControls = (joyX != 0 || joyY != 0 || joyR != 0) ? 0 : enabledDriveControls;
        if(enabledDriveControls != 0) {
            robot.logging.removeLog("Average wheel power");
            return;
        }

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
