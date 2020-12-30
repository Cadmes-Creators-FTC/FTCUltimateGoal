package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

@Disabled
public class Logging extends RobotComponent {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    private Map<String, String> logs;

    public Logging(Telemetry inputTelemetry, MainRobot inputRobot) {
        super(inputTelemetry, inputRobot);

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

    public void setLog(String key, String value){
        logs.put(key, value);
    }
    public void removeLog(String key){
        logs.remove(key);
    }

    public void updateLogs(int delay) throws InterruptedException {
        while (robot.isRunning){
            for(Map.Entry<String, String> entry : logs.entrySet()){
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();

            Thread.sleep(delay);
        }
    }
}
