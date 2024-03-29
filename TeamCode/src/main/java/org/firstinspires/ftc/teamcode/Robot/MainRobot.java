package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public Intake intake;
    public Conveyor conveyor;
    public RingStackDetection ringStackDetection;
    public ArrayList<RobotComponent> componentsList = new ArrayList<RobotComponent>();

    public LinearOpMode opMode;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry, String[] inputEnabledComponents, LinearOpMode inputOpmode) {
        opMode = inputOpmode;

        List<String> enabledComponents = Arrays.asList(inputEnabledComponents);

        if(enabledComponents.contains("logging")) {
            logging = new Logging(inputTelemetry, this);
            componentsList.add(logging);
        }
        if(enabledComponents.contains("gyroscope")) {
            gyroscope = new Gyroscope(hardwareMap, this);
            componentsList.add(gyroscope);
        }
        if(enabledComponents.contains("driving")) {
            driving = new Driving(hardwareMap, this);
            componentsList.add(driving);
        }
        if(enabledComponents.contains("shooter")) {
            shooter = new Shooter(hardwareMap, this);
            componentsList.add(shooter);
        }
        if(enabledComponents.contains("wobbleArm")) {
            wobbleArm = new WobbleArm(hardwareMap, this);
            componentsList.add(wobbleArm);
        }
        if(enabledComponents.contains("intake")) {
            intake = new Intake(hardwareMap, this);
            componentsList.add(intake);
        }
        if(enabledComponents.contains("conveyor")) {
            conveyor = new Conveyor(hardwareMap, this);
            componentsList.add(conveyor);
        }
        if(enabledComponents.contains("ringStackHeightDetection")) {
            ringStackDetection = new RingStackDetection(hardwareMap, this);
            componentsList.add(ringStackDetection);
        }
    }

    public void checkForStop() throws InterruptedException{
        while(isRunning){
            if(opMode.isStopRequested())
                stopRobot();

            Thread.sleep(50);
        }
    }

    public void startThreads(){
        new Thread(){
            @Override
            public void run(){
                try {
                    checkForStop();
                } catch (InterruptedException ignored) { }
            }
        }.start();

        for(int i = 0; i < componentsList.size(); i++){
            componentsList.get(i).startThreads();
        }
    }

    public void stopRobot(){
        isRunning = false;
    }
}
