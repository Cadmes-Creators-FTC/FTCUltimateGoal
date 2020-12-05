package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.misc.MathFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class MainRobot {
    //hardwareMap and telemetry
    private Telemetry telemetry;

    public Boolean isRunning = true;

    public Driving driving;

    public DcMotor shooterWheelL;
    public DcMotor shooterWheelR;

    //IMU
    private final BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle;
    private double targetAngle;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry) throws InterruptedException {
        telemetry = inputTelemetry;

        //assign imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        driving = new Driving(hardwareMap, telemetry, this);

//        shooterWheelL = hardwareMap.get(DcMotor.class, "ShooterL");
//        shooterWheelR = hardwareMap.get(DcMotor.class, "ShooterR");


        new Thread(){
            @Override
            public void run(){
                try {
                    KeepCurrentAngleUpdated();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }


    //region IMU callibration
    public void WaitForGyroCalibration() throws InterruptedException{
        while (!imu.isGyroCalibrated()) {
            Thread.sleep(50);
        }
    }
    //endregion

    //region Angles
    private void UpdateCurrentAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        deltaAngle *= -1;

        currentAngle += deltaAngle;

        currentAngle = MathFunctions.clambAngleDegrees(currentAngle);

        lastAngles = angles;
    }
    private void KeepCurrentAngleUpdated() throws InterruptedException {
        while (isRunning){
            UpdateCurrentAngle();
            Thread.sleep(100);
        }
    }
    public void ResetCurrentAngle(){
        currentAngle = 0;
        targetAngle = 0;
        lastAngles = new Orientation();
    }

    public double getCurrentAngle(){
        return currentAngle;
    }
    public double getTargetAngle() { return targetAngle; }
    public void setTargetAngle(double newTargetAngle) {
        targetAngle = MathFunctions.clambAngleDegrees(newTargetAngle);
    }
    //endregion
}
