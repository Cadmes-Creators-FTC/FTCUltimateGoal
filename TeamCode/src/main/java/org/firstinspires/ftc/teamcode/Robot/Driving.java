package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Matrix;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPowerConfig;

@Disabled
public class Driving {
    private final Telemetry telemetry; // for logging and debugging
    private final MainRobot robot; //reference to robot

    private final DcMotor wheelLF;
    private final DcMotor wheelRF;
    private final DcMotor wheelRB;
    private final DcMotor wheelLB;
    private final int ticksPerRotation = 1120;

    private boolean keepAtTargetAngle = false;

    private Vector2 currentPosition = new Vector2(0, 0);
    private WheelPosition currentWheelPosTicks = new WheelPosition(0, 0, 0, 0);

    public Driving(HardwareMap hardwareMap, Telemetry inputTelemetry, MainRobot inputRobot) {
        telemetry = inputTelemetry;
        robot = inputRobot;

        wheelLF = hardwareMap.get(DcMotor.class, "LFWheel");
        wheelRF = hardwareMap.get(DcMotor.class, "RFWheel");
        wheelRB = hardwareMap.get(DcMotor.class, "RBWheel");
        wheelLB = hardwareMap.get(DcMotor.class, "LBWheel");

        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //start threads
        new Thread(){
            @Override
            public void run(){
                try {
                    KeepAtTargetAngle();
                } catch (InterruptedException ignored) { }
            }
        }.start();
        new Thread(){
            @Override
            public void run(){
                try {
                    KeepPositionUpdated();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }

    //region teleOp wheelPowers
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

    //region keep target angle
    public void setKeepAtTargetAngle(boolean x){
        keepAtTargetAngle = x;
    }
    private void KeepAtTargetAngle() throws InterruptedException {
        while (robot.isRunning){
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

        double targetAngle = robot.gyroscope.getTargetAngle();
        double currentAngle = robot.gyroscope.getCurrentAngle();
        if (Math.abs(targetAngle - currentAngle) > minDegreesOff)
            correction = targetAngle - currentAngle;
        correction = correction * gain;

        return correction;
    }
     //endregion

    //region Position
    public Vector2 getCurrentPosition(){
        return currentPosition;
    }
    public void setCurrentPosition(Vector2 pos){
        currentPosition = pos;
    }

    public void KeepPositionUpdated() throws InterruptedException{
        while (robot.isRunning){
            /* get and update wheel tick positions */
            WheelPosition wheelPosDelta = new WheelPosition(
                    wheelLF.getCurrentPosition()*-1 - currentWheelPosTicks.lf,
                    wheelRF.getCurrentPosition()*-1 - currentWheelPosTicks.rf,
                    wheelRB.getCurrentPosition()*-1 - currentWheelPosTicks.rb,
                    wheelLB.getCurrentPosition()*-1 - currentWheelPosTicks.lb
            );
            currentWheelPosTicks = new WheelPosition(
                    wheelLF.getCurrentPosition()*-1,
                    wheelRF.getCurrentPosition()*-1,
                    wheelRB.getCurrentPosition()*-1,
                    wheelLB.getCurrentPosition()*-1
            );

            telemetry.addData("pos x", currentPosition.x);
            telemetry.addData("pos y", currentPosition.y);
            telemetry.addData("ticks 1", currentWheelPosTicks.lf);
            telemetry.addData("ticks 2", currentWheelPosTicks.rf);
            telemetry.addData("ticks 3", currentWheelPosTicks.rb);
            telemetry.addData("ticks 4", currentWheelPosTicks.lb);
            telemetry.addData("rot", robot.gyroscope.getCurrentAngle());
            telemetry.update();

            /* get wheel pos matrix */
            wheelPosDelta.ToCM(36, ticksPerRotation);
            Matrix wheelPosMatrix = new Matrix(1, 4, new double[][]{
                    { wheelPosDelta.lf, wheelPosDelta.rf, wheelPosDelta.rb, wheelPosDelta.lb }
            });

            /* get transformation matrix */
            double angle = Math.toRadians(robot.gyroscope.getCurrentAngle()) + Math.PI/4;
            double sinVal = Math.sqrt(2)*Math.sin(angle);
            double cosVal = Math.sqrt(2)*Math.cos(angle);
            Matrix transformMatrix = new Matrix(4, 2, new double[][]{
                    { sinVal,  cosVal },
                    { cosVal, -sinVal },
                    { sinVal,  cosVal },
                    { cosVal, -sinVal },
            });
            transformMatrix = Matrix.scale(transformMatrix, 0.25);

            /* get movement */
            Matrix posMatrix = Matrix.multiply(wheelPosMatrix, transformMatrix);
            Vector2 deltaPos = new Vector2(posMatrix.matrix[0][1], posMatrix.matrix[0][0]);

//            /* transform individual wheel movement to whole robot movement */
//            double cornerDegrees = 90/(Math.sqrt(2)+1);
//            Vector2 wheelVectorRight = new Vector2(-Math.sin(cornerDegrees), 1.53*Math.cos(cornerDegrees));
//            Vector2 wheelVectorLeft = new Vector2(Math.sin(cornerDegrees), 1.53*Math.cos(cornerDegrees));
//
//
//            Vector2 vectorLF = Vector2.Multiply(wheelVectorRight, wheelPosDelta.lf);
//            Vector2 vectorRF = Vector2.Multiply(wheelVectorLeft, wheelPosDelta.rf);
//            Vector2 vectorRB = Vector2.Multiply(wheelVectorRight, wheelPosDelta.rb);
//            Vector2 vectorLB = Vector2.Multiply(wheelVectorLeft, wheelPosDelta.lb);
//
//            Vector2 vectorFront = Vector2.Add(vectorLF, vectorRF);
//            Vector2 vectorBack = Vector2.Add(vectorLB, vectorRB);
//            Vector2 deltaPos = Vector2.Add(vectorFront, vectorBack);
//
//            // divide by 4 to get average vector
//            deltaPos = Vector2.Divide(deltaPos, 4);
//
//            /* account for rotation */
//            double currentAngle = robot.gyroscope.getCurrentAngle();
//            Vector2 t_deltaPos = deltaPos;
//            deltaPos.x = Math.sin(currentAngle+90)*t_deltaPos.x + Math.sin(currentAngle)*t_deltaPos.y;
//            deltaPos.y = Math.cos(currentAngle+90)*t_deltaPos.x + Math.cos(currentAngle)*t_deltaPos.y;

            /* update position */
            currentPosition = Vector2.Add(currentPosition, deltaPos);

            /* timeout between updates */
            Thread.sleep(30);
        }
    }
    public void DriveToPosition (Vector2 targetPos) throws InterruptedException {
        double stopDistance = 10;

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

            deltaPos = Vector2.Subtract(targetPos, currentPosition);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }

    //endregion
}
