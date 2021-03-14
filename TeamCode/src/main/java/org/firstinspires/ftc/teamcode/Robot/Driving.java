package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Misc.DataTypes.Matrix;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;
import org.firstinspires.ftc.teamcode.Misc.MiniPID;

@Disabled
public class Driving extends RobotComponent {
    private final DcMotor wheelLF;
    private final DcMotor wheelRF;
    private final DcMotor wheelRB;
    private final DcMotor wheelLB;
    private final int ticksPerRotation = 1120;

    private Vector2 currentPosition = new Vector2(0, 0);
    private WheelPosition currentWheelPosTicks = new WheelPosition(0, 0, 0, 0);

    public Driving(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        wheelLF = hardwareMap.get(DcMotor.class, "LFWheel");
        wheelRF = hardwareMap.get(DcMotor.class, "RFWheel");
        wheelRB = hardwareMap.get(DcMotor.class, "RBWheel");
        wheelLB = hardwareMap.get(DcMotor.class, "LBWheel");

        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        setWheelRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setWheelZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void startThreads(){
        new Thread(){
            @Override
            public void run(){
                try {
                    keepPositionUpdated();
                } catch (InterruptedException ignored) { }
            }
        }.start();
    }


    public void setWheelRunMode(DcMotor.RunMode mode){
        wheelLF.setMode(mode);
        wheelRF.setMode(mode);
        wheelRB.setMode(mode);
        wheelLB.setMode(mode);
    }
    public void setWheelZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour){
        wheelLF.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelRF.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelRB.setZeroPowerBehavior(zeroPowerBehaviour);
        wheelLB.setZeroPowerBehavior(zeroPowerBehaviour);
    }


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


    public WheelPosition getWheelTicks() {
        return new WheelPosition(
                wheelLF.getCurrentPosition()*-1,
                wheelRF.getCurrentPosition()*-1,
                wheelRB.getCurrentPosition()*-1,
                wheelLB.getCurrentPosition()*-1
        );
    }
    public void resetWheelTicks(){
        setWheelRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public Vector2 getCurrentPosition(){
        return currentPosition;
    }
    public void setCurrentPosition(Vector2 pos){
        currentPosition = pos;
    }

    public void keepPositionUpdated() throws InterruptedException{
        while (robot.isRunning){
            /* get and update wheel tick positions */
            WheelPosition wheelPosDelta = WheelPosition.subtract(getWheelTicks(), currentWheelPosTicks);
            currentWheelPosTicks = getWheelTicks();

            /* get wheel pos matrix */
            wheelPosDelta.toCM(10*Math.PI, ticksPerRotation);

            /* get movement */
            double yScaler = 1.52;
            double xScaler = 0.57;

            double deltaX = ((wheelPosDelta.lf + wheelPosDelta.rb) - (wheelPosDelta.rf + wheelPosDelta.lb)) / 4 * 1.5;
            double deltaY = (wheelPosDelta.lf + wheelPosDelta.rf + wheelPosDelta.rb + wheelPosDelta.lb) / 4;
            Matrix deltaPos = new Matrix(new double[][]{{deltaX*xScaler, deltaY*yScaler}});

            double angleRad = Math.toRadians(robot.gyroscope.getCurrentAngle());
            Matrix rotMatrix = new Matrix(new double[][]{
                    { Math.cos(angleRad), -Math.sin(angleRad) },
                    { Math.sin(angleRad), Math.cos(angleRad) },
            });

            deltaPos = Matrix.multiply(deltaPos, rotMatrix);

            /* update position */
            currentPosition = Vector2.add(currentPosition, deltaPos.toVector2());

            /* timeout between updates */
            Thread.sleep(50);
        }
    }

    public void driveToPositionForwardOnly(Vector2 targetPos, Double targetAngle, double speedScaler) throws InterruptedException{
        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);

        double angle = (Math.toDegrees(Math.atan2(deltaPos.y, deltaPos.x)) - 90) * -1;

        rotateToAngleFixed(angle);

        driveToPosition(targetPos, null, speedScaler);

        if(targetAngle != null)
            robot.driving.rotateToAngleFixed(targetAngle);
    }
    public void driveToPosition(Vector2 targetPos, Double targetAngle, double speedScaler) throws InterruptedException {
        if(targetAngle != null)
            robot.gyroscope.setTargetAngle(targetAngle);

        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);
        double totalDistance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));

        double previousDistance = 0;

        /* pid */
        double integralScaler = 0.005;
        double maxAccelerationPercentile = 0.5;
        double AccelerationDist = 50;
        double accelerationBarrier = AccelerationDist;
        double decelerationBarrier = totalDistance - Math.min(maxAccelerationPercentile*totalDistance, AccelerationDist);
        double maxAngleCorrection = 0.25;
        double minSpeedForMovement = 0.1;

        double speed = 0;
        double distance = totalDistance;

        double stopDistance = 5;
        while (robot.isRunning && (distance > stopDistance)){
            /* debugging */
            robot.logging.setLog("driveToPos-dist", distance);
            robot.logging.setLog("driveToPos-distTotal", totalDistance);
            robot.logging.setLog("driveToPos-accelerationBarrier", accelerationBarrier);
            robot.logging.setLog("driveToPos-pos", getCurrentPosition());
            robot.logging.setLog("driveToPos-speed", speed);
            robot.logging.setLog("driveToPos-speedAfterScale", speed*speedScaler);

            /* pid */
            double traveledDistance = totalDistance-distance;
            double previousTraveledDistance = totalDistance-previousDistance;

            //accelerate/decelerate
            if(distance != previousDistance){
                if(traveledDistance >= decelerationBarrier){
                    double remainingSpeedDecrease = speed;

                    double currentMovementPercentage = (traveledDistance - previousTraveledDistance)/(totalDistance - previousTraveledDistance);
                    currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                    speed -= remainingSpeedDecrease*currentMovementPercentage;
                }else if(traveledDistance <= accelerationBarrier){
                    double remainingSpeedIncrease = 1-speed;

                    double currentMovementPercentage = (traveledDistance - previousTraveledDistance)/(accelerationBarrier - previousTraveledDistance);
                    currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                    speed += remainingSpeedIncrease*currentMovementPercentage;
                }
            }else{ // scale speed if not moved
                speed = MathFunctions.clamp(integralScaler+speed, 0, 1);
            }


            /* drive towards targetPos with speed from pid */
            double angleRad = Math.toRadians(robot.gyroscope.getCurrentAngle());
            Matrix rotMatrix = new Matrix(new double[][]{
                    { Math.cos(-angleRad), -Math.sin(-angleRad) },
                    { Math.sin(-angleRad), Math.cos(-angleRad)  },
            });
            Vector2 relativeDeltaPos = Matrix.multiply(deltaPos.toMatrix(), rotMatrix).toVector2();

            WheelPowerConfig wpc = new WheelPowerConfig(
                    relativeDeltaPos.y + relativeDeltaPos.x,
                    relativeDeltaPos.y - relativeDeltaPos.x,
                    relativeDeltaPos.y + relativeDeltaPos.x,
                    relativeDeltaPos.y - relativeDeltaPos.x
            );
            wpc.clampScale();

            double angleCorrection = MathFunctions.clamp(getAngleCorrection(), -maxAngleCorrection, maxAngleCorrection);
            WheelPowerConfig angleCorrectionWPC = new WheelPowerConfig(
                    angleCorrection,
                    -angleCorrection,
                    -angleCorrection,
                    angleCorrection
            );
            angleCorrectionWPC.clamp();

            wpc = WheelPowerConfig.add(wpc, angleCorrectionWPC);
            wpc.clampScale();

            double scaledSpeed = ((speed*(1-minSpeedForMovement))+minSpeedForMovement)*speedScaler;
            wpc = WheelPowerConfig.multiply(wpc, scaledSpeed);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            previousDistance = distance;
            deltaPos = Vector2.subtract(targetPos, currentPosition);
            distance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));
        }
        /* debugging */
        robot.logging.removeLog("driveToPos-dist");
        robot.logging.removeLog("driveToPos-distTotal");
        robot.logging.removeLog("driveToPos-accelerationBarrier");
        robot.logging.removeLog("driveToPos-pos");
        robot.logging.removeLog("driveToPos-speed");
        robot.logging.removeLog("driveToPos-speedAfterScale");

        if(targetAngle != null)
            rotateToAngleFixed(targetAngle);

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    public void rotateToAngle(double targetAngle, double speedScaler) throws InterruptedException {
        robot.gyroscope.setTargetAngle(targetAngle);

        double deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
        double totalAngle = Math.abs(deltaAngle);

        double previousAngle = 0;

        /* pid */
        double integralScaler = 0.005;
        double maxAccelerationPercentile = 0.5;
        double AccelerationAngle = 20;
        double accelerationBarrier = AccelerationAngle;
        double decelerationBarrier = totalAngle - Math.min(maxAccelerationPercentile*totalAngle, AccelerationAngle);
        double minSpeedForMovement = 0.15;

        double speed = 0;
        double angle = totalAngle;

        double stopAngle = 4;
        while (robot.isRunning && (angle > stopAngle)){
            /* debugging */
            robot.logging.setLog("rotateToAngle-angle", angle);
            robot.logging.setLog("rotateToAngle-angleTotal", totalAngle);
            robot.logging.setLog("rotateToAngle-deltaAngle", angle);
            robot.logging.setLog("rotateToAngle-accelerationBarrier", accelerationBarrier);
            robot.logging.setLog("rotateToAngle-rot", robot.gyroscope.getCurrentAngle());
            robot.logging.setLog("rotateToAngle-speed", speed);

            /* pid */
            double traveledAngle = totalAngle-angle;
            double previousTraveledAngle= totalAngle-previousAngle;

            //accelerate/decelerate
            if(angle != previousAngle) {
                if (traveledAngle >= decelerationBarrier) {
                    double remainingSpeedDecrease = speed;

                    double currentMovementPercentage = (traveledAngle - previousTraveledAngle) / (totalAngle - previousTraveledAngle);
                    currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                    speed -= remainingSpeedDecrease * currentMovementPercentage;
                }else if(traveledAngle <= accelerationBarrier){
                    double remainingSpeedIncrease = 1 - speed;

                    double currentMovementPercentage = (traveledAngle - previousTraveledAngle) / (accelerationBarrier - previousTraveledAngle);
                    currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                    speed += remainingSpeedIncrease * currentMovementPercentage;
                }
            }else { // scale speed if not moved
                speed = MathFunctions.clamp(integralScaler+speed, 0, 1);
            }


            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaAngle,
                    -deltaAngle,
                    -deltaAngle,
                    deltaAngle
            );
            wpc.clampScale();

            double scaledSpeed = ((speed*(1-minSpeedForMovement))+minSpeedForMovement)*speedScaler;
            robot.logging.setLog("rotateToAngle-speedAfterScale", scaledSpeed);
            wpc = WheelPowerConfig.multiply(wpc, scaledSpeed);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            previousAngle = angle;
            deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
            angle = Math.abs(deltaAngle);
        }
        /* debugging */
        robot.logging.removeLog("rotateToAngle-angle");
        robot.logging.removeLog("rotateToAngle-angleTotal");
        robot.logging.removeLog("rotateToAngle-deltaAngle");
        robot.logging.removeLog("rotateToAngle-accelerationBarrier");
        robot.logging.removeLog("rotateToAngle-rot");
        robot.logging.removeLog("rotateToAngle-speed");
        robot.logging.removeLog("rotateToAngle-speedAfterScale");

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    public void rotateToAngleFixed(double targetAngle) throws InterruptedException{
        robot.gyroscope.setTargetAngle(targetAngle);

        double deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle()-robot.gyroscope.getCurrentAngle());

        double stopAngle = 5;
        while (robot.isRunning && Math.abs(deltaAngle) > stopAngle){
            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaAngle,
                    -deltaAngle,
                    -deltaAngle,
                    deltaAngle
            );
            wpc.clampScale();

            wpc = WheelPowerConfig.multiply(wpc, 0.4);
            setWheelPowers(wpc);

            Thread.sleep(50);

            deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle()-robot.gyroscope.getCurrentAngle());
            robot.logging.setLog("deltaAngle", deltaAngle);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }

    private double getAngleCorrection(){
        double scaler = 0.05;

        double targetAngle = robot.gyroscope.getTargetAngle();
        double currentAngle = robot.gyroscope.getCurrentAngle();
        double deltaAngle = targetAngle - currentAngle;

        return deltaAngle*scaler;
    }
}
