package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "wheels_only", group = "TeleOp2")
public class WheelsOnlyTeleOp extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] disabledComponents = {"shooter", "wobbleArm"};
        robot = new MainRobot(hardwareMap, telemetry, disabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        while (opModeIsActive()){
            driveWithJoystick();
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
        robot.driving.setWheelPowers(wpc);

        robot.logging.setLog("power", (wpc.lf+wpc.rf+wpc.rb+wpc.lb)/4);
    }
}