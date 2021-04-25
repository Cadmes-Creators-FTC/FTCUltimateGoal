package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Shooter extends RobotComponent {
    private final DcMotor shooterWheelL;
    private final DcMotor shooterWheelR;

    double tickRateL = 0;
    double tickRateR = 0;

    private int targetTickRate = 0;
    private final boolean debug = false;

    double errorSumL = 0;
    double errorSumR = 0;
    double lastErrorL = 0;
    double lastErrorR = 0;

    public Shooter(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        shooterWheelL = hardwareMap.get(DcMotor.class, "ShooterL");
        shooterWheelR = hardwareMap.get(DcMotor.class, "ShooterR");

        shooterWheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterWheelL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void startThreads(){
        new Thread(){
            @Override
            public void run(){
                try {
                    keepTickRateUpdated();
                } catch (InterruptedException ignored) { }
            }
        }.start();
        new Thread(){
            @Override
            public void run(){
                try {
                    PIDLoop();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }

    public void keepTickRateUpdated() throws InterruptedException{
        double lastPosL = shooterWheelL.getCurrentPosition();
        double lastPosR = shooterWheelR.getCurrentPosition();

        double lastTimeStamp = (((double)System.currentTimeMillis())/1000);

        while (robot.isRunning){
            double dt = (((double)System.currentTimeMillis())/1000) - lastTimeStamp;
            lastTimeStamp = (((double)System.currentTimeMillis())/1000);

            tickRateL = (shooterWheelL.getCurrentPosition()-lastPosL)/dt;
            lastPosL = shooterWheelL.getCurrentPosition();
            tickRateR = (shooterWheelR.getCurrentPosition()-lastPosR)/dt;
            lastPosR = shooterWheelL.getCurrentPosition();

            Thread.sleep(10);
        }
    }

    public void PIDLoop() throws InterruptedException{
        double errorL = tickRateL - targetTickRate;
        double errorR = tickRateR - targetTickRate;

        double kP = 0;
        double kI = 0;
        double iLimit = 0;
        double kD = 0;

        errorSumL = 0;
        errorSumR = 0;
        double lastTimeStamp = (((double)System.currentTimeMillis())/1000);
        lastErrorL = errorL;
        lastErrorR = errorR;

        while (robot.isRunning){
            double dt = (((double)System.currentTimeMillis())/1000) - lastTimeStamp;
            lastTimeStamp = (((double)System.currentTimeMillis())/1000);

            if(errorL < iLimit)
                errorSumL += dt*errorL;
            if(errorR < iLimit)
                errorSumR += dt*errorR;

            double errorRateL = (errorL-lastErrorL)/dt;
            double errorRateR = (errorR-lastErrorR)/dt;

            double speedL = kP*errorL + kI*errorSumL + kD*errorRateL;
            double speedR = kP*errorR + kI*errorSumR + kD*errorRateR;

            shooterWheelL.setPower(speedL);
            shooterWheelR.setPower(speedR);

            Thread.sleep(20);

            lastErrorL = errorL;
            lastErrorR = errorR;
            errorL = tickRateL - targetTickRate;
            errorR = tickRateR - targetTickRate;

            if(debug)
                robot.logging.setLog("shooterAverageTickRate", (tickRateL + tickRateR) / 2);
        }
    }

    public void resetPIDLoop(){
        double errorL = tickRateL - targetTickRate;
        double errorR = tickRateR - targetTickRate;

        errorSumL = 0;
        errorSumR = 0;
        lastErrorL = errorL;
        lastErrorR = errorR;
    }

    public void turnOn(int tickRate){
        targetTickRate = tickRate;
        resetPIDLoop();

        if(debug)
            robot.logging.setLog("shooterTargetTickRate", targetTickRate);
    }
    public void turnOff(){
        targetTickRate = 0;
        resetPIDLoop();

        if(debug)
            robot.logging.setLog("shooterTargetTickRate", 0);
    }

    public  boolean isOn(){
        return shooterWheelL.getPower() != 0;
    }
}
