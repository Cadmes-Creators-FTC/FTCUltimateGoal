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

        arm = hardwareMap.get(Servo.class, "wobbleArm");
        arm.scaleRange(0, 0.75);

        gripper = hardwareMap.get(Servo.class, "wobbleGripper");
        gripper.scaleRange(0.2, 0.6);
        gripper.setPosition(0);
    }

    public void armUp(){
        double armPos = arm.getPosition();
        arm.setPosition(armPos + 0.005);
    }
    public void armDown(){
        double armPos = arm.getPosition();
        arm.setPosition(armPos - 0.005);
    }

    public void closeGripper(){
        gripper.setPosition(0);
    }
    public void openGripper(){
        gripper.setPosition(1);
    }
    public boolean isGripperOpen(){
        return gripper.getPosition() != 0;
    }

    public void toStopPosition(){
        arm.scaleRange(0, 1);
        arm.setPosition(1);
    }
}
