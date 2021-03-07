package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;

import java.util.HashMap;
import java.util.TreeMap;

@Disabled
public class Logging extends RobotComponent {
    //hardwareMap and telemetry
    private final Telemetry telemetry;

    private TreeMap<String, Object> logs = new TreeMap<String, Object>();
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
        formatSetLog(key, value);
    }
    public void removeLog(String key){
        logs.remove(key);
    }

    public void clearLogs(){
        logs.clear();
    }

    private void formatSetLog(String key, Object value){
        if(value.getClass() == WheelPosition.class){
            WheelPosition castValue = (WheelPosition) value;

            logs.put(key+"-lf", castValue.lf);
            logs.put(key+"-rf", castValue.rf);
            logs.put(key+"-rb", castValue.rb);
            logs.put(key+"-lb", castValue.lb);
        }
        else if(value.getClass() == WheelPowerConfig.class){
            WheelPowerConfig castValue = (WheelPowerConfig) value;

            logs.put(key+"-lf", castValue.lf);
            logs.put(key+"-rf", castValue.rf);
            logs.put(key+"-rb", castValue.rb);
            logs.put(key+"-lb", castValue.lb);
        }else if(value.getClass() == Vector2.class){
            Vector2 castValue = (Vector2) value;

            logs.put(key+"-x", castValue.x);
            logs.put(key+"-y", castValue.y);
        }
        else
            logs.put(key, value);
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
