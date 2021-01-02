package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class MainRobot {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    public Boolean isRunning = true;

    public Driving driving;
    public Shooter shooter;
    public Gyroscope gyroscope;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry) throws InterruptedException {
        telemetry = inputTelemetry;

//        gyroscope = new Gyroscope(hardwareMap, telemetry, this);
//        driving = new Driving(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
    }

    public void startThreats(){
        gyroscope.startThreats();
        driving.startThreats();
//        shooter.startThreats();
    }
}
