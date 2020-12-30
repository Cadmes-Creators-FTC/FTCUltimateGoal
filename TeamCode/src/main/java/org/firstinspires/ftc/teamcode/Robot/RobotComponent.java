package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotComponent {
    protected final Telemetry telemetry; // for logging and debugging
    public final MainRobot robot; //reference to robot

    public RobotComponent(Telemetry inputTelemetry, MainRobot inputRobot) {
        telemetry = inputTelemetry;
        robot = inputRobot;
    }

    public void startThreads(){

    }
}
