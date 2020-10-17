package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfigs.MainRobotConfig;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPowerConfig;

@TeleOp(name = "Main_Robot", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    private MainRobotConfig robot;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new MainRobotConfig(hardwareMap, telemetry);
        //wait for imu to calibrate
        robot.WaitForGyroCalibration();

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            DriveWithController();
        }
    }

    private void DriveWithController(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        //reverse y joystick
        joyY *= -1;
        telemetry.addData("joyY", joyY);

        //create wheel power config
        WheelPowerConfig wpc = new WheelPowerConfig(
                joyY + joyX + joyR,
                joyY - joyX - joyR,
                joyY + joyX - joyR,
                joyY - joyX + joyR
        );
        wpc.clamp();


        //smooth out the acceleration
        wpc.lf = 0.6*Math.pow(wpc.lf, 3) + 0.4*wpc.lf;
        wpc.rf = 0.6*Math.pow(wpc.rf, 3) + 0.4*wpc.rf;
        wpc.rb = 0.6*Math.pow(wpc.rb, 3) + 0.4*wpc.rb;
        wpc.lb = 0.6*Math.pow(wpc.lb, 3) + 0.4*wpc.lb;
        robot.setWheelPowers(wpc);
        RingIntake();
    }
    //voor de intake van de ringen
    private void RingIntake(){
        boolean buttonA = gamepad2.a;
        if (buttonA == true);
            robot.intakeWheel.setPower(0.5);
    }
}
