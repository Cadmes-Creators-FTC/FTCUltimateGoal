package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Intake extends RobotComponent {
    private final DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
    }

    @Override
    public void startThreads(){

    }

    public void turnOn(){
        intakeMotor.setPower(1);
    }
    public void turnOff(){
        intakeMotor.setPower(0);
    }
}
