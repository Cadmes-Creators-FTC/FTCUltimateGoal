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
    public Shooter shooter;
    public Gyroscope gyroscope;

    public MainRobot(HardwareMap hardwareMap, Telemetry inputTelemetry) throws InterruptedException {
        telemetry = inputTelemetry;

        driving = new Driving(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
        gyroscope = new Gyroscope(hardwareMap, telemetry, this);
    }
}
