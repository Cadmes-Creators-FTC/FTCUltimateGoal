package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "wheelsOnly", group = "TestingTeleOps")
public class WheelsOnly extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
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

            robot.logging.setLog("wheelTicks", robot.driving.getWheelTicks());
            robot.logging.setLog("wheelPowers", robot.driving.getWheelPowers());
        }
    }

    int enabledDriveControls = 0;
    private void driveWithJoystick(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;
        robot.logging.setLog("joyX", joyX);
        robot.logging.setLog("joyY", joyY);

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

        double averageWheelPower = (Math.abs(wpc.lf) + Math.abs(wpc.rf) + Math.abs(wpc.rb) + Math.abs(wpc.lb)) / 4;

        //smooth out the acceleration - f(x) = 0.6x^2 + 0.4x
        double scalerVal = 0.6*Math.pow(averageWheelPower, 2) + 0.4*averageWheelPower;
        wpc = WheelPowerConfig.multiply(wpc, scalerVal);

        robot.driving.setWheelPowers(wpc);
    }

    private void driveWithDpad(){
        enabledDriveControls = (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left) ? 1 : enabledDriveControls;
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
        }else {
            WheelPowerConfig wpc = new WheelPowerConfig(0, 0, 0, 0);
            robot.driving.setWheelPowers(wpc);
        }
    }
}
