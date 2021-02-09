package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;

@Disabled
public class Gyroscope extends RobotComponent{
    private final BNO055IMU imu;

    private Orientation lastAngles = new Orientation();

    private double currentAngle = 0;
    private double targetAngle = 0;

    public Gyroscope(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    @Override
    public void startThreads(){
        new Thread(){
            @Override
            public void run(){
                try {
                    keepCurrentAngleUpdated();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }

    public void waitForGyroCalibration() throws InterruptedException{
        while (!imu.isGyroCalibrated()) {
            Thread.sleep(50);
        }
    }

    private void keepCurrentAngleUpdated() throws InterruptedException {
        while (robot.isRunning){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
            deltaAngle *= -1;

            currentAngle += deltaAngle;

            currentAngle = MathFunctions.clambAngleDegrees(currentAngle);
            lastAngles = angles;

            Thread.sleep(50);
        }
    }

    public double getCurrentAngle(){
        return currentAngle;
    }
    public void setCurrentAngle(double angle){
        currentAngle = angle;
        targetAngle = angle;
        lastAngles = new Orientation();
    }

    public double getTargetAngle() { return targetAngle; }
    public void setTargetAngle(double newTargetAngle) {
        targetAngle = MathFunctions.clambAngleDegrees(newTargetAngle);
    }
}
