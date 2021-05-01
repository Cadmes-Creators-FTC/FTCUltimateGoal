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
        Vector2 wobbleDropPoint = new Vector2(180, 190);//200
        double hoek = 135;
        if(numOfRings == 1) {
            hoek = 145;
            wobbleDropPoint = new Vector2(120, 220);//110 290
        }
        else if(numOfRings == 4) {
            hoek = 90;
            wobbleDropPoint = new Vector2(180, 320);//200
        }

        robot.shooter.turnOn(0.49);
        robot.driving.driveToPositionYOnly(new Vector2(150, 70), 180.0, 0.6, null);

        robot.conveyor.turnOn(0.75);
        Thread.sleep(3000);
        robot.conveyor.turnOff();
        robot.shooter.turnOff();


        //drop off first wobble goal
        if(numOfRings != 0){
            robot.driving.driveToPositionYOnly(new Vector2(210, 120), null, 1.0, null);
        }
        robot.intake.turnOn();
        robot.driving.driveToPositionYOnly(wobbleDropPoint, hoek, 1.0, -1.0);
        robot.intake.turnOff();
        robot.wobbleArm.releaseWobbleAutonomous();
        Thread.sleep(400);

        robot.driving.driveToPositionYOnly(new Vector2(100, 200), null, 1.0, null);
    }
}