package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="RemoteRed", group="RedAutonomous")
public class RemoteRed extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

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
        Vector2 wobbleDropPoint = new Vector2(200, 170);
        if(numOfRings == 1)
            wobbleDropPoint = new Vector2(140, 230);
        else if(numOfRings == 4)
            wobbleDropPoint = new Vector2(200, 290);

        robot.wobbleArm.armDownAutonomous(2500);
        robot.wobbleArm.armUpAutonomous(500);
        //drop off first wobble goal
        robot.driving.driveToPositionForwardOnly(new Vector2(210, 120), null, 0.75);
        robot.driving.driveToPositionForwardOnly(wobbleDropPoint, 30.0, 0.75);
        robot.wobbleArm.armDownAutonomous(400);
        robot.wobbleArm.openGripperAutonomous(400);

        //shoot rings
        robot.driving.driveToPositionForwardOnly(new Vector2(150, 180), 180.0, 0.75);
        robot.shooter.turnOn(0.92);
        robot.conveyor.turnOn(1);
        Thread.sleep(1000);
        robot.shooter.turnOff();
        robot.conveyor.turnOff();

//        //pick up second wobble goal
//        robot.driving.driveToPositionForwardOnly(new Vector2(130, 100), null, 0.75);
//        robot.wobbleArm.closeGripperAutonomous();
//        robot.wobbleArm.armUpAutonomous();
//
//        //drop off second wobble goal
//        robot.driving.driveToPositionForwardOnly(new Vector2(200, 170), null, 0.75);
//        robot.wobbleArm.openGripperAutonomous();
//        robot.wobbleArm.toStopPosition();

        //drive to launch line
        robot.driving.driveToPositionForwardOnly(new Vector2(100, 210), null, 0.75);
    }
}
