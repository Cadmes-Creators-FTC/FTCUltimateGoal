package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Disabled
public class MainRobot {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    public Boolean isRunning = true;

    public Driving driving;
    public Gyroscope gyroscope;
    public Shooter shooter;
    public ArrayList<RobotComponent> componentsList;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputDisabledComponents) {
        telemetry = inputTelemetry;

        List<String> disabledComponents = Arrays.asList(inputDisabledComponents);
        if(!disabledComponents.contains("gyroscope")) {
            gyroscope = new Gyroscope(hardwareMap, telemetry, this);
            componentsList.add(gyroscope);
        }
        if(!disabledComponents.contains("driving")) {
            driving = new Driving(hardwareMap, telemetry, this);
            componentsList.add(driving);
        }
        if(!disabledComponents.contains("shooter")) {
            shooter = new Shooter(hardwareMap, telemetry, this);
            componentsList.add(shooter);
        }
    }

    public void startThreads(){
        for(int i = 0; i < componentsList.size(); i++){
            componentsList.get(i).startThreads();
        }
    }
}
