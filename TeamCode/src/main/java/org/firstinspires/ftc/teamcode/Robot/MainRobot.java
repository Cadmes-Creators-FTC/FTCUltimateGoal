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
<<<<<<< HEAD

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry) throws InterruptedException {
        telemetry = inputTelemetry;

//        gyroscope = new Gyroscope(hardwareMap, telemetry, this);
//        driving = new Driving(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
=======
    public Shooter shooter;
    public Logging logging;
    public ArrayList<RobotComponent> componentsList;

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
>>>>>>> 2fa3ffdf8b3e02a419d3bc5e724d048397098eaa
    }

    public void startThreads(){
        for(int i = 0; i < componentsList.size(); i++){
            componentsList.get(i).startThreads();
        }
    }
}
