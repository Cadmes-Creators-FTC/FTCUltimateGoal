package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Disabled
public class MainRobot {
    public Boolean isRunning = true;

    public Driving driving;
    public Gyroscope gyroscope;
    public Shooter shooter;
    public Logging logging;
    public WobbleArm wobbleArm;
    public ArrayList<RobotComponent> componentsList = new ArrayList<RobotComponent>();

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputDisabledComponents) {
        List<String> disabledComponents = Arrays.asList(inputDisabledComponents);

        if(!disabledComponents.contains("logging")) {
            logging = new Logging(inputTelemetry, this);
            componentsList.add(logging);
        }
        if(!disabledComponents.contains("gyroscope")) {
            gyroscope = new Gyroscope(hardwareMap, this);
            componentsList.add(gyroscope);
        }
        if(!disabledComponents.contains("driving")) {
            driving = new Driving(hardwareMap, this);
            componentsList.add(driving);
        }
        if(!disabledComponents.contains("shooter")) {
            shooter = new Shooter(hardwareMap, this);
            componentsList.add(shooter);
        }
        if(!disabledComponents.contains("wobbleArm")) {
            wobbleArm = new WobbleArm(hardwareMap, this);
            componentsList.add(wobbleArm);
        }
    }

    public void startThreads(){
        for(int i = 0; i < componentsList.size(); i++){
            componentsList.get(i).startThreads();
        }
    }
}
