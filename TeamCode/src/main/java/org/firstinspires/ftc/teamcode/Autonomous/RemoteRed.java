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
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter","intake", "conveyor", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.resetWheelTicks();
        robot.driving.setCurrentPosition(new Vector2(170, 40));
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
        Vector2 wobbleDropPoint = new Vector2(180, 170);//200
        double hoek = 20;
        if(numOfRings == 1) {
            hoek = -20;
            wobbleDropPoint = new Vector2(180, 230);//110 290
        }
        else if(numOfRings == 4) {
            hoek = 30;
            wobbleDropPoint = new Vector2(180, 290);//200
        }

        robot.wobbleArm.armDownAutonomous(2500);
        robot.wobbleArm.armUpAutonomous(500);
        //drop off first wobble goal
        robot.driving.driveToPositionYOnly(new Vector2(210, 120), null, 0.80, null);
        robot.driving.driveToPositionYOnly(wobbleDropPoint, hoek, 0.80, null);
        robot.wobbleArm.armDownAutonomous(400);
        robot.wobbleArm.openGripperAutonomous(400);
        robot.wobbleArm.armUpAutonomous(400);

        //shoot rings
        robot.driving.driveToPositionYOnly(new Vector2(150, 180), 180.0, 1, null);
        robot.shooter.turnOn(0.92);
        robot.conveyor.turnOn(1);
        Thread.sleep(2000);//1500
        robot.shooter.turnOff();
        robot.conveyor.turnOff();

        //wobbelgoal
        /*robot.driving.driveToPositionForwardOnly(new Vector2(125, 140), -180.0, 1);
        robot.wobbleArm.armDownAutonomous(400);
        robot.driving.driveToPositionForwardOnly(new Vector2(125, 130), null, 0.75);*/

        //drive to launch line
        robot.driving.driveToPositionYOnly(new Vector2(100, 210), null, 1, null);
    }
}
