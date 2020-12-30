package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotComponent {
    public final MainRobot robot; //reference to robot

    public RobotComponent(Telemetry inputTelemetry, MainRobot inputRobot) {
        robot = inputRobot;
    }

    public void startThreads(){

    }
}
