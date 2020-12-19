package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPowerConfig;

@TeleOp(name = "testEncoder", group = "TeleOp")
public class TestEncoder extends LinearOpMode {

    DcMotor motor;
    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            motor.setPower(1);

            telemetry.addData("val", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);

    }
}
