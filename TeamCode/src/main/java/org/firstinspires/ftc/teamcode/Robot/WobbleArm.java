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
    }
    public void armDown(){

    }

    public void closeHand(){

    }
    public void openHand(){

    }
}
