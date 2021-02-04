package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Shooter extends RobotComponent {
    private final DcMotor shooterWheelL;
    private final DcMotor shooterWheelR;

    public Shooter(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

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
    public void turnOff(){
        shooterWheelL.setPower(0);
        shooterWheelR.setPower(0);
    }

    public  boolean isOn(){
        return shooterWheelL.getPower() != 0;
    }
}
