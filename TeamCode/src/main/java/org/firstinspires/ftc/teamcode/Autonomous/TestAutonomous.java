package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="blue")
public class TestAutonomous extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        String[] disabledComponents = {};
        robot = new MainRobot(hardwareMap, telemetry, disabledComponents);

        robot.gyroscope.waitForGyroCalibration();
        robot.driving.setCurrentPosition(new Vector2(0, 0));
        robot.startThreads();


        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        //start autonomous
        //AutonomousSequence();
        while (!isStopRequested()){
            autonomousSequence();

            telemetry.addData("State", "Done");
            telemetry.update();
        }
    }

    //autonomous sequence
    private void autonomousSequence() throws InterruptedException {
        robot.driving.driveToPosition(new Vector2(0, 100));
    }
}
