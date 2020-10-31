package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfigs.MainRobotConfig;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;

@Autonomous(name="TestAutonomous", group="blue")
public class TestAutonomous extends LinearOpMode {
    private MainRobotConfig robot;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("State", "Initializing");
        telemetry.update();

        //initialize robot hardware
        /*robot = new MainRobotConfig(hardwareMap, telemetry);
        robot.setCurrentPosition(new Vector2(0, 0));
        //wait for imu to calibrate
        robot.WaitForGyroCalibration();*/

        telemetry.addData("State", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        //start autonomous
        //AutonomousSequence();
        while (!isStopRequested()){
        testAutomous();

        telemetry.addData("State", "Done");
        telemetry.update();
    }
    }

    private void testAutomous() {
        DcMotor testWheelL;
        DcMotor testWheelR;

        testWheelL = hardwareMap.get(DcMotor.class, "TwheelL");
        testWheelR = hardwareMap.get(DcMotor.class, "TwheelR");

        testWheelR.setPower(-1);
        testWheelL.setPower(1);
    }

    //autonomous sequence
    private void AutonomousSequence() throws InterruptedException {
        robot.DriveToPosition(new Vector2(0, 100));
    }

}
