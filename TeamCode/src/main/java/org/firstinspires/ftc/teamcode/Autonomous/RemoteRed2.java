package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="RemoteRed2", group="")
public class RemoteRed2 extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter","intake", "conveyor", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.resetWheelTicks();
        robot.driving.setCurrentPosition(new Vector2(170, 20));
        robot.gyroscope.setCurrentAngle(-90);
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();

        robot.logging.clearLogs();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        double numOfRings = robot.ringStackDetection.getStackSize();
        robot.logging.setLog("numOfRings", numOfRings);
        Vector2 wobbleDropPoint = new Vector2(180, 185);//200
        Vector2 wobbleDropPointArm = new Vector2(160, 190);
        double wobbleDropHoek = 135;
        double wobbleDropHoekArm = 90;

        if(numOfRings == 1) {
            wobbleDropHoek = 145;
            wobbleDropPoint = new Vector2(120, 250);//110 290
        }
        else if(numOfRings == 4) {
            wobbleDropHoek = 135;
            wobbleDropPoint = new Vector2(180, 320);//200
        }

        robot.shooter.turnOn(0.49);
        if(numOfRings != 0) {
            robot.driving.driveToPositionYOnly(new Vector2(210, 160), null, 0.6, null);
            robot.driving.driveToPositionYOnly(new Vector2(135, 165), 180.0, 0.6, null);
        }else{
            robot.driving.driveToPositionYOnly(new Vector2(150, 160), 180.0, 1.0, null);
        }

        robot.conveyor.turnOn(0.75);
        Thread.sleep(3000);
        robot.conveyor.turnOff();
        robot.shooter.turnOff();


        //drop off first wobble goal
        robot.driving.driveToPositionYOnly(wobbleDropPoint, wobbleDropHoek, 1.0, -1.0);
        robot.wobbleArm.releaseWobbleAutonomous();
        Thread.sleep(400);

        if(numOfRings == 0){
            robot.wobbleArm.armToPickup();
            robot.wobbleArm.openGripper();
            robot.driving.driveToPositionYOnly(new Vector2(115, 140), 180.0, 0.7, null);
            robot.driving.driveToPositionYOnly(new Vector2(robot.driving.getCurrentPosition().x, 80), 180.0, 0.3, 1.0);
            Thread.sleep(100);
            robot.wobbleArm.closeGripper();
            Thread.sleep(800);
            robot.wobbleArm.armToHold();

            robot.driving.driveToPositionYOnly(wobbleDropPointArm, wobbleDropHoekArm, 1.0, null);
            robot.wobbleArm.armToBottom();
            Thread.sleep(500);
            robot.wobbleArm.openGripper();
            Thread.sleep(500);
            robot.wobbleArm.armToTop();
        }

        double toLineSpeed = numOfRings == 0 ? 1.0 : 0.6;
        robot.driving.driveToPositionYOnly(new Vector2(100, 200), 180.0, toLineSpeed, null);
    }
}