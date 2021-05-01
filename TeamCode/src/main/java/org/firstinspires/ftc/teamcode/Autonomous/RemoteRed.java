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
            wobbleDropPoint = new Vector2(160, 230);//110 290
        }
        else if(numOfRings == 4) {
            hoek = 30;
            wobbleDropPoint = new Vector2(180, 290);//200
        }

        robot.intake.turnOn();
        Thread.sleep(500);
        robot.intake.turnOff();

        robot.wobbleArm.armDownAutonomous(2500);
        robot.wobbleArm.armUpAutonomous(500);
        //drop off first wobble goal
        if(numOfRings != 0){
            robot.driving.driveToPositionYOnly(new Vector2(210, 120), null, 0.55, 1.0);
        }
        robot.driving.driveToPositionYOnly(wobbleDropPoint, hoek, 0.55, null);
        robot.wobbleArm.armDownAutonomous(400);
        robot.wobbleArm.openGripperAutonomous(400);
        robot.wobbleArm.armUpAutonomous(400);

        //shoot rings
        robot.driving.driveToPositionYOnly(new Vector2(147, 180), null, 1, null);
        robot.driving.rotateToAngle(165, 1);
        robot.driving.rotateToAngle(180, 0.2);
        robot.shooter.turnOn(0.93);
        robot.conveyor.turnOn(0.5);
        Thread.sleep(2000);//1500
        robot.shooter.turnOff();
        robot.conveyor.turnOff();

        if(numOfRings == 1){
            robot.intake.turnOn();
            robot.conveyor.turnOn(0.6);
            robot.driving.driveToPositionYOnly(new Vector2(robot.driving.getCurrentPosition().x, 150), null, 0.3, 1.0);

            robot.driving.driveToPositionYOnly(new Vector2(robot.driving.getCurrentPosition().x, 180), null, 1, -1.0);
            robot.driving.rotateToAngle(165, 1);
            robot.driving.rotateToAngle(180, 0.2);
            robot.shooter.turnOn(0.93);
            robot.conveyor.turnOn(0.5);
            while (getRuntime() < 29){ Thread.sleep(50); }
            robot.shooter.turnOff();
            robot.conveyor.turnOff();
        }

        //drive to launch line
        robot.driving.driveToPositionYOnly(new Vector2(147, 210), null, 1, -1.0);
    }
}
