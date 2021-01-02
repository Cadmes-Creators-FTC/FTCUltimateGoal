package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Disabled
public class Logging extends RobotComponent {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    private HashMap<String, Object> logs = new HashMap<String, Object>();
    private int updateDelay = 100;

    public Logging(Telemetry inputTelemetry, MainRobot inputRobot) {
        super(inputRobot);

        telemetry = inputTelemetry;
    }

    @Override
    public void startThreads(){
        new Thread(){
            @Override
            public void run(){
                try {
                    updateLogs();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }

    public void setLog(String key, Object value){
        logs.put(key, value);
    }
    public void removeLog(String key){
        logs.remove(key);
    }

    public void updateLogs() throws InterruptedException {
        while (robot.isRunning){
            for(HashMap.Entry<String, Object> entry : logs.entrySet()){
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();

            Thread.sleep(updateDelay);
        }
    }
}
