package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.misc.DataTypes.WheelPowerConfig;

@Disabled
public class Driving {
    private Telemetry telemetry; // for logging and debugging
    private MainRobot robot; //reference to robot

    private final DcMotor wheelLF;
    private final DcMotor wheelRF;
    private final DcMotor wheelRB;
    private final DcMotor wheelLB;
    private final int ticksPerRotation = 1680;

    private boolean keepAtTargetAngle = false;

    private Vector2 currentPosition = new Vector2(0, 0);
    private WheelPosition currentPositionTicks = new WheelPosition(0, 0, 0, 0);

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

        double targetAngle = robot.getTargetAngle();
        double currentAngle = robot.getCurrentAngle();
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
            /* get position delta and update wheel ticks */
            WheelPosition wheelPosDelta = new WheelPosition(
                    wheelLF.getCurrentPosition() - currentPositionTicks.lf,
                    wheelRF.getCurrentPosition() - currentPositionTicks.rf,
                    wheelRB.getCurrentPosition() - currentPositionTicks.rb,
                    wheelLB.getCurrentPosition() - currentPositionTicks.lb
            );
            currentPositionTicks = new WheelPosition(
                    wheelLF.getCurrentPosition(),
                    wheelRF.getCurrentPosition(),
                    wheelRB.getCurrentPosition(),
                    wheelLB.getCurrentPosition()
            );

            telemetry.addData("pos x", currentPosition.x);
            telemetry.addData("pos y", currentPosition.y);
            telemetry.addData("ticks 1", currentPositionTicks.lf);
            telemetry.addData("ticks 2", currentPositionTicks.rf);
            telemetry.addData("ticks 3", currentPositionTicks.rb);
            telemetry.addData("ticks 4", currentPositionTicks.lb);
            telemetry.addData("rot", robot.getCurrentAngle());
            telemetry.update();

            /* transform ticks to cm */
            wheelPosDelta.ToCM(10*Math.PI, ticksPerRotation);

            /* transform individual wheel movement to whole robot movement */
            double cornerDegrees = 90/(Math.sqrt(2)+1);
            Vector2 wheelVectorRight = new Vector2(Math.sin(cornerDegrees), Math.cos(cornerDegrees));
            Vector2 wheelVectorLeft = new Vector2(-Math.sin(cornerDegrees), Math.cos(cornerDegrees));

            Vector2 vectorLF = Vector2.Multiply(wheelVectorRight, wheelPosDelta.lf);
            Vector2 vectorRF = Vector2.Multiply(wheelVectorLeft, wheelPosDelta.rf);
            Vector2 vectorRB = Vector2.Multiply(wheelVectorRight, wheelPosDelta.rb);
            Vector2 vectorLB = Vector2.Multiply(wheelVectorLeft, wheelPosDelta.lb);

            Vector2 vectorFront = Vector2.Add(vectorLF, vectorRF);
            Vector2 vectorBack = Vector2.Add(vectorLB, vectorRB);
            Vector2 deltaPos = Vector2.Add(vectorFront, vectorBack);

            // divide by 4 to get average vector
            deltaPos = Vector2.Divide(deltaPos, 4);

            /* account for rotation */
            double currentAngle = robot.getCurrentAngle();
            Vector2 t_deltaPos = deltaPos;
            deltaPos.x = Math.sin(currentAngle+90)*t_deltaPos.x + Math.sin(currentAngle)*t_deltaPos.y;
            deltaPos.y = Math.cos(currentAngle+90)*t_deltaPos.x + Math.cos(currentAngle)*t_deltaPos.y;

            /* update position */
            currentPosition = Vector2.Add(currentPosition, deltaPos);

            /* timeout between updates */
            Thread.sleep(30);
        }
    }
//    public void DriveToPosition (Vector2 targetPos) throws InterruptedException {
//        double stopDistance = 10;
//        WheelPosition prevWheelTicks = new WheelPosition(0, 0, 0, 0);
//
//        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Vector2 deltaPos = Vector2.Subtract(targetPos, currentPosition);
//        while((Math.abs(deltaPos.x) > stopDistance || Math.abs(deltaPos.y) > stopDistance)) {
//            WheelPowerConfig wpc = new WheelPowerConfig(
//                    deltaPos.y + deltaPos.x,
//                    deltaPos.y - deltaPos.x,
//                    deltaPos.y + deltaPos.x,
//                    deltaPos.y - deltaPos.x
//            );
//
//            wpc.clamp();
//            setWheelPowers(wpc);
//
//            telemetry.addData("lf power", wpc.lf);
//            telemetry.addData("rf power", wpc.rf);
//            telemetry.addData("rb power", wpc.rb);
//            telemetry.addData("lb power", wpc.lb);
//
//            telemetry.addData("pos x", currentPosition.x);
//            telemetry.addData("pos y", currentPosition.y);
//            telemetry.update();
//
//            Thread.sleep(100);
//
//            WheelPosition wheelTicks = new WheelPosition(
//                    wheelLF.getCurrentPosition(),
//                    wheelRF.getCurrentPosition(),
//                    wheelRB.getCurrentPosition(),
//                    wheelLB.getCurrentPosition()
//            );
//            WheelPosition wheelTicksDelta = WheelPosition.Subtract(wheelTicks, prevWheelTicks);
//            prevWheelTicks = wheelTicks;
//
////            Vector2 posChange = WheelTicksToPos(wheelTicksDelta);
////            currentPosition = Vector2.Add(currentPosition, posChange);
//
//            deltaPos = Vector2.Subtract(targetPos, currentPosition);
//        }
//
//        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
//    }
    //endregion
}
