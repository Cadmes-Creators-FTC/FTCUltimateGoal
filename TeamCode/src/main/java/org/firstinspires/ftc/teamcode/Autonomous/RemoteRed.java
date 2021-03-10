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
        // put arm in autonomous up pos to extend arm
        robot.wobbleArm.armUpAutonomous();

        robot.driving.driveToPositionForwardOnly(new Vector2(200, 170), null, 0.75);
        robot.wobbleArm.armDownAutonomous();
        robot.wobbleArm.openGripperAutonomous();

//        robot.driving.driveToPosition(new Vector2(110, 90), 180.0, 1);
////        robot.driving.driveToPositionForwardOnly(new Vector2(110, 90), 180.0, 1);
//        robot.wobbleArm.closeGripperAutonomous();
//        robot.wobbleArm.armUpAutonomous();
//
//        robot.driving.driveToPosition(new Vector2(180, 190), 45.0, 1);
////        robot.driving.driveToPositionForwardOnly(new Vector2(180, 190), 45.0, 1);
//        robot.wobbleArm.armDownAutonomous();
//        robot.wobbleArm.openGripperAutonomous();
//        robot.wobbleArm.armUpAutonomous();
//
//        robot.driving.driveToPosition(new Vector2(120, 210), 45.0, 1);
////        robot.driving.driveToPositionForwardOnly(new Vector2(120, 210), 45.0, 1);
    }
}
