package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Disabled
public class WobbleArm extends RobotComponent {
    private Servo arm;
    private Servo gripper;

    public WobbleArm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);


    }

    @Override
    public void startThreads(){

    }

    public void armUp(){
        double armPos = arm.getPosition();
        arm.setPosition(armPos - 0.01);

    }
    public void armDown(){
        double armPos = arm.getPosition();
        arm.setPosition(armPos + 0.01);
    }

    public void closeGripper(){

    }
    public void openGripper(){

    }
}
