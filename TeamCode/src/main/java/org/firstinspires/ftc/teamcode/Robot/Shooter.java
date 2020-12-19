package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Shooter {
    private Telemetry telemetry; // for logging and debugging
    private MainRobot robot; //reference to robot

    private final DcMotor shooterWheelL;
    private final DcMotor shooterWheelR;

    public Shooter(HardwareMap hardwareMap, Telemetry inputTelemetry, MainRobot inputRobot) {
        telemetry = inputTelemetry;
        robot = inputRobot;

        shooterWheelL = hardwareMap.get(DcMotor.class, "ShooterL");
        shooterWheelR = hardwareMap.get(DcMotor.class, "ShooterR");
    }

    public void TurnOn(double power){
        shooterWheelL.setPower(power);
        shooterWheelR.setPower(power);
    }
    public void TurnOf(){
        shooterWheelL.setPower(0);
        shooterWheelR.setPower(0);
    }
}
