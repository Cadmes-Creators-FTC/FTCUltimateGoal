package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Shooter extends RobotComponent {
    private final DcMotor shooterWheelL;
    private final DcMotor shooterWheelR;

    public Shooter(HardwareMap hardwareMap, Telemetry inputTelemetry, MainRobot inputRobot) {
        super(inputTelemetry, inputRobot);

        shooterWheelL = hardwareMap.get(DcMotor.class, "ShooterL");
        shooterWheelR = hardwareMap.get(DcMotor.class, "ShooterR");

        shooterWheelL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void startThreads(){

    }

    public void turnOn(double power){
        shooterWheelL.setPower(power);
        shooterWheelR.setPower(power);
    }
    public void turnOf(){
        shooterWheelL.setPower(0);
        shooterWheelR.setPower(0);
    }
}
