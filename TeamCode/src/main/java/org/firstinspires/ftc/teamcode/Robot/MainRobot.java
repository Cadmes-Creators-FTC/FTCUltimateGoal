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

    public ArrayList<RobotComponent> components;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputDisabledComponents) {
        telemetry = inputTelemetry;

        List<String> disabledComponents = Arrays.asList(inputDisabledComponents);
        if(!disabledComponents.contains("gyroscope"))
            components.add( new Gyroscope(hardwareMap, telemetry, this) );
        if(!disabledComponents.contains("driving"))
            components.add( new Driving(hardwareMap, telemetry, this) );
        if(!disabledComponents.contains("shooter"))
            components.add( new Shooter(hardwareMap, telemetry, this) );
    }

    public void startThreads(){
        for(int i = 0; i < components.size(); i++){
            components.get(i).startThreads();
        }
    }
}
