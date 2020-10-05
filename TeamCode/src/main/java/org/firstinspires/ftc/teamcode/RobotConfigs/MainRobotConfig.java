package org.firstinspires.ftc.teamcode.RobotConfigs;

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
public class MainRobotConfig {
    //hardwareMap and telemetry
    private Telemetry telemetry;

    //motors
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    public DcMotor intakeWheel;

    private Vector2 currentPosition = new Vector2(0, 0);

    //IMU
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle;
    private boolean keepAtTargetAngle = false;
    private double targetAngle;

    public MainRobotConfig(HardwareMap hardwareMap, Telemetry inputTelemetry) throws InterruptedException {
        telemetry = inputTelemetry;

        //assign imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        wheelLF = hardwareMap.get(DcMotor.class, "LFWheel");
        wheelRF = hardwareMap.get(DcMotor.class, "RFWheel");
        wheelRB = hardwareMap.get(DcMotor.class, "RBWheel");
        wheelLB = hardwareMap.get(DcMotor.class, "LBWheel");
        intakeWheel = hardwareMap.get(DcMotor.class,"Intake");

        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        wheelLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Thread keepAtTargetAngleThread = new Thread(){
            @Override
            public void run(){
                try {
                    KeepAtTargetAngle();
                } catch (InterruptedException ignored) { }
            }
        };
        keepAtTargetAngleThread.start();
    }


    //region IMU callibration
    public void WaitForGyroCalibration() throws InterruptedException{
        while (!imu.isGyroCalibrated()) {
            Thread.sleep(50);
        }
    }
    //endregion

    //region Angles
    public void UpdateCurrentAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        deltaAngle *= -1;

        currentAngle += deltaAngle;

        currentAngle = MathFunctions.clambAngleDegrees(currentAngle);

        lastAngles = angles;
    }
    public void ResetCurrentAngle(){
        currentAngle = 0;
        targetAngle = 0;
        lastAngles = new Orientation();
    }

    public double getCurrentAngle(){
        return currentAngle;
    }
    public void setTargetAngle(double newTargetAngle) {
        targetAngle = MathFunctions.clambAngleDegrees(newTargetAngle);
    }
    //endregion

    //region Position
    public Vector2 getCurrentPosition(){
        return currentPosition;
    }
    public void setCurrentPosition(Vector2 pos){
        currentPosition = pos;
    }
    //endregion

    //region WheelPowerConfig
    public void setWheelPowers(WheelPowerConfig wheelPowerConfig){
        //set motor power
        wheelLF.setPower(wheelPowerConfig.lf);
        wheelRF.setPower(wheelPowerConfig.rf);
        wheelRB.setPower(wheelPowerConfig.rb);
        wheelLB.setPower(wheelPowerConfig.lb);
    }
    public WheelPowerConfig getWheelPowers(){
        return new WheelPowerConfig(
                wheelLF.getPower(),
                wheelRF.getPower(),
                wheelRB.getPower(),
                wheelLB.getPower()
        );
    }
    //endregion

    //region KeepAtTargetAngle
    public void setKeepAtTargetAngle(boolean x){
        keepAtTargetAngle = x;
    }
    private void KeepAtTargetAngle() throws InterruptedException {
        //noinspection InfiniteLoopStatement
        while (true){
            if (keepAtTargetAngle){
                double correction = getAngleWheelCorrection();
                WheelPowerConfig currentWPC = getWheelPowers();
                WheelPowerConfig correctionWPC = new WheelPowerConfig(correction, -correction, - correction, correction);

                WheelPowerConfig newWPC = currentWPC.Add(correctionWPC);

                setWheelPowers(newWPC);
            }else
                Thread.sleep(500);
        }
    }
    private double getAngleWheelCorrection(){
        double gain = .05;
        double minDegreesOff = 3;

        double correction = 0;
        UpdateCurrentAngle();

        if (Math.abs(targetAngle - currentAngle) > minDegreesOff)
            correction = targetAngle - currentAngle;
        correction = correction * gain;

        return correction;
    }
    //endregion

    public void DriveToPosition (Vector2 targetPos) {
        Vector2 deltaPos = targetPos.Subtract(currentPosition);
        WheelPosition prevWheelPositions = new WheelPosition(0, 0, 0, 0);
        double stopDistance = 5;

        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(Math.abs(deltaPos.x) > stopDistance || Math.abs(deltaPos.y) > stopDistance) {
            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaPos.y + deltaPos.x,
                    deltaPos.y - deltaPos.x,
                    deltaPos.y + deltaPos.x,
                    deltaPos.y - deltaPos.x
            );
            wpc.clamp();
            setWheelPowers(wpc);

            WheelPosition currentWheelPosition = new WheelPosition(
                    wheelLF.getCurrentPosition(),
                    wheelRF.getCurrentPosition(),
                    wheelRB.getCurrentPosition(),
                    wheelLB.getCurrentPosition()
            );
            WheelPosition wheelPositionDelta = currentWheelPosition.Subtract(prevWheelPositions);
            prevWheelPositions = currentWheelPosition;

            Vector2 posChange = WheelTicksToPos(wheelPositionDelta);
            currentPosition.Add(posChange);

            deltaPos = targetPos.Subtract(currentPosition);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    private Vector2 WheelTicksToPos(WheelPosition wheelPos){
        wheelPos.ToCM(10*Math.PI, 1120);

        Vector2 vectorLF = new Vector2(1/Math.sqrt(2), 1).Multipy(wheelPos.lf);
        Vector2 vectorRF = new Vector2(-1/Math.sqrt(2), 1).Multipy(wheelPos.rf);
        Vector2 vectorRB = new Vector2(1/Math.sqrt(2), 1).Multipy(wheelPos.rb);
        Vector2 vectorLB = new Vector2(-1/Math.sqrt(2), 1).Multipy(wheelPos.lb);

        return vectorLF.Add(vectorRF).Add(vectorRB).Add(vectorLB);
    }
}
