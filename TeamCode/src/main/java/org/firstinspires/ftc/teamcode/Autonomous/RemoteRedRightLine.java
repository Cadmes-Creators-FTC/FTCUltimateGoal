package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="RemoteRedRightLine", group="RedAutonomous")
public class RemoteRedRightLine extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "ringStackHeightDetection"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(191, 37.5));
        robot.gyroscope.setCurrentAngle(-90);
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");

        waitForStart();

        robot.logging.setLog("state", "Running");

        autonomousSequence();

        robot.isRunning = false;

        robot.logging.setLog("state", "Stopped");
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        int stackHeight = robot.ringStackDetection.getStackSize();
        // 22    20.4

        robot.driving.driveToPosition(new Vector2(193, 132), 0, 0.5);

        Vector2 targetZonePosition;
        if(stackHeight == 0)
            targetZonePosition = new Vector2(179, 179);
        else if(stackHeight == 1)
            targetZonePosition = new Vector2(119, 239);
        else
            targetZonePosition = new Vector2(179, 298);
        robot.driving.driveToPosition(targetZonePosition, 70, 0.5);

        robot.wobbleArm.armDown();
        robot.wobbleArm.openGripper();
        robot.wobbleArm.toStopPosition();

        robot.driving.driveToPosition(new Vector2(193, 132), 0, 0.5);
        robot.driving.driveToPosition(new Vector2(179, 45), 0, 0.5);
    }
}
