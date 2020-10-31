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

    public DcMotor intakeWheelL;
    public DcMotor intakeWheelR;

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
        intakeWheelL = hardwareMap.get(DcMotor.class,"IntakeL");
        intakeWheelR = hardwareMap.get(DcMotor.class, "IntakeR");

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
                WheelPowerConfig correctionWPC = new WheelPowerConfig(correction, -correction, -correction, correction);

                WheelPowerConfig newWPC = WheelPowerConfig.Add(currentWPC, correctionWPC);

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

    //region Autonomous driving to position
    public void DriveToPosition (Vector2 targetPos) throws InterruptedException {
        double stopDistance = 10;
        WheelPosition prevWheelTicks = new WheelPosition(0, 0, 0, 0);

        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Vector2 deltaPos = Vector2.Subtract(targetPos, currentPosition);
        while((Math.abs(deltaPos.x) > stopDistance || Math.abs(deltaPos.y) > stopDistance)) {
            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaPos.y + deltaPos.x,
                    deltaPos.y - deltaPos.x,
                    deltaPos.y + deltaPos.x,
                    deltaPos.y - deltaPos.x
            );
            wpc.clamp();
            setWheelPowers(wpc);

            Thread.sleep(100);

            WheelPosition wheelTicks = new WheelPosition(
                    wheelLF.getCurrentPosition(),
                    wheelRF.getCurrentPosition(),
                    wheelRB.getCurrentPosition(),
                    wheelLB.getCurrentPosition()
            );
            WheelPosition wheelTicksDelta = WheelPosition.Subtract(wheelTicks, prevWheelTicks);
            prevWheelTicks = wheelTicks;

            Vector2 posChange = WheelTicksToPos(wheelTicksDelta);
            currentPosition = Vector2.Add(currentPosition, posChange);

            deltaPos = Vector2.Subtract(targetPos, currentPosition);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    private Vector2 WheelTicksToPos(WheelPosition wheelPos){
        wheelPos.ToCM(10*Math.PI, 1120);

        Vector2 vectorLF = Vector2.Multiply(new Vector2(1/Math.sqrt(2), 1), wheelPos.lf);
        Vector2 vectorRF = Vector2.Multiply(new Vector2(-1/Math.sqrt(2), 1), wheelPos.rf);
        Vector2 vectorRB = Vector2.Multiply(new Vector2(1/Math.sqrt(2), 1), wheelPos.rb);
        Vector2 vectorLB = Vector2.Multiply(new Vector2(-1/Math.sqrt(2), 1), wheelPos.lb);

        Vector2 total = Vector2.Add(Vector2.Add(vectorLF, vectorRF), Vector2.Add(vectorLB, vectorRB));
        total = Vector2.Divide(total, 4);

        return total;
    }
    //endregion
}
