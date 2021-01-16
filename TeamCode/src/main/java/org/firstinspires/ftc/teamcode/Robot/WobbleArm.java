package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Disabled
public class WobbleArm extends RobotComponent {
    private Servo arm;
    private Servo hand;

    public WobbleArm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);


        // hier servos toewijzen met hardwaremap.get
    }

    @Override
    public void startThreads(){

    }

    public void armUp(){
        // in deze functies de arm en hand omhoog en omlaag doen.
        double armPos = arm.getPosition();
        arm.setPosition(armPos - 0.1);

    }
    public void armDown(){
        double armPos = arm.getPosition();
        arm.setPosition(armPos + 0.1);

    }

    public void closeHand(){
        //in deze funtie gaat de gripper open

    }
    public void openHand(){

    }
}
