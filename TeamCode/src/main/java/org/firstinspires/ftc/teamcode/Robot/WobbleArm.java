package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class WobbleArm extends RobotComponent {
    public Servo arm;
    private Servo gripper;

    private Servo autonomousReleaser;

    public WobbleArm(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        arm = hardwareMap.get(Servo.class, "wobbleArm1");
        arm.scaleRange(0, 0.85);

        gripper = hardwareMap.get(Servo.class, "wobbleGripper");
        gripper.scaleRange(0.1, 0.6);
        closeGripper();

        autonomousReleaser = hardwareMap.get(Servo.class, "wobbleAutonomousReleaser");
        autonomousReleaser.scaleRange(0, 0.9);
        autonomousReleaser.setPosition(0);
    }

    public void releaseWobbleAutonomous(){
        autonomousReleaser.setPosition(1);
    }

    public void armUpAutonomous(int delay){
        arm.setPosition(0.4);
        try{
            Thread.sleep(delay);
        } catch (InterruptedException ignored){}
    }
    public void armDownAutonomous(int delay){
        arm.setPosition(0.9);
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

    public void armToBottom(){
        arm.setPosition(0);
    }
    public void armToPickup(){
        arm.setPosition(0.08);
    }
    public void armToHold(){
        arm.setPosition(0.32);
    }
    public void armToDrop(){
        arm.setPosition(0.67);
    }
    public void armToTop(){
        arm.setPosition(1);
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
