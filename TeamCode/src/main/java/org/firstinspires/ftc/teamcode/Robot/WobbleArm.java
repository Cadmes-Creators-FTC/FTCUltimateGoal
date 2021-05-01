package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Disabled
public class WobbleArm extends RobotComponent {
    private Servo arm1;
    private Servo arm2;
    private Servo gripper;

    private Servo autonomousReleaser;

    public WobbleArm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        arm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        arm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        arm1.scaleRange(0, 0.85);
        arm2.scaleRange(0, 0.85);
        arm2.setDirection(Servo.Direction.REVERSE);

        gripper = hardwareMap.get(Servo.class, "wobbleGripper");
        gripper.scaleRange(0.1, 0.6);//0,25, 0,6
        closeGripper();

        autonomousReleaser = hardwareMap.get(Servo.class, "wobbleAutonomousReleaser");
        autonomousReleaser.scaleRange(0, 0.9);
        autonomousReleaser.setPosition(0);
    }

    public void releaseWobbleAutonomous(){
        autonomousReleaser.setPosition(1);
    }

    public void armUpAutonomous(int delay){
        arm1.setPosition(0.4);
        arm2.setPosition(0.4);
        try{
            Thread.sleep(delay);
        } catch (InterruptedException ignored){}
    }
    public void armDownAutonomous(int delay){
        arm1.setPosition(0.9);
        arm2.setPosition(0.9);
        try{
            Thread.sleep(delay);
        } catch (InterruptedException ignored){}
    }
    public void closeGripperAutonomous(int delay){
        gripper.setPosition(0);
        try{
            Thread.sleep(delay);
        } catch (InterruptedException ignored){}
    }
    public void openGripperAutonomous(int delay){
        gripper.setPosition(1);
        try{
            Thread.sleep(delay);
        } catch (InterruptedException ignored){}
    }

    public void armUp(){
        double armPos = arm1.getPosition();
        arm1.setPosition(armPos - 0.008);
        arm2.setPosition(armPos - 0.008);
    }
    public void armDown(){
        double armPos = arm1.getPosition();
        arm1.setPosition(armPos + 0.008 );
        arm2.setPosition(armPos + 0.008 );
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

}
