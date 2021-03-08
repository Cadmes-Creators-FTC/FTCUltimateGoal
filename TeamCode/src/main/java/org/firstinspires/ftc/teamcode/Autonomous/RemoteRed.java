package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="RemoteRedRightLine", group="RedAutonomous")
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
        // put arm in autonomous up pos to extend arm
        robot.wobbleArm.armUpAutonomous();

        robot.driving.driveToPosition(new Vector2(210, 180), 45.0, 0.5);
        robot.wobbleArm.armDownAutonomous();
        robot.wobbleArm.openGripperAutonomous();

        robot.driving.driveToPosition(new Vector2(110, 80), 180.0, 0.5);
        robot.wobbleArm.closeGripperAutonomous();
        robot.wobbleArm.armUpAutonomous();

        robot.driving.driveToPosition(new Vector2(190, 180), 45.0, 0.5);
        robot.wobbleArm.armDownAutonomous();
        robot.wobbleArm.openGripperAutonomous();

        robot.driving.driveToPosition(new Vector2(100, 210), 45.0, 0.5);
    }
}