package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Conveyor extends RobotComponent {
    private final DcMotor conveyorMotor;

    public Conveyor(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        conveyorMotor = hardwareMap.get(DcMotor.class, "Conveyor");
        conveyorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void startThreads(){

    }

    public boolean isOn(){
        return conveyorMotor.getPower() != 0;
    }

    public void turnOn(){
        conveyorMotor.setPower(1);
    }
    public void turnOff(){
        conveyorMotor.setPower(0);
    }
}
