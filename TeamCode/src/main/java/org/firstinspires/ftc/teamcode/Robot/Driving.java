package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Misc.DataTypes.Matrix;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;

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

        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            double yScaler = 1.5;

            double deltaX = ((wheelPosDelta.lf + wheelPosDelta.rb) - (wheelPosDelta.rf + wheelPosDelta.lb)) / 4 * 1.5;
            double deltaY = (wheelPosDelta.lf + wheelPosDelta.rf + wheelPosDelta.rb + wheelPosDelta.lb) / 4;
            Matrix deltaPos = new Matrix(new double[][]{{deltaX, deltaY*yScaler}});

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

    public void driveToPositionForwardOnly(Vector2 targetPos, Double targetRotation, double speedScaler) throws InterruptedException{
        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);

        double angle = (Math.toDegrees(Math.atan2(deltaPos.y, deltaPos.x)) - 90) * -1;

        robot.gyroscope.setTargetAngle(angle);
        rotateToTargetAngle(speedScaler);

        driveToPosition(targetPos, angle, speedScaler);

        if(targetRotation != null){
            robot.gyroscope.setTargetAngle(targetRotation);
            rotateToTargetAngle(speedScaler);
        }
    }
    public void driveToPosition(Vector2 targetPos, double targetRotation, double speedScaler) throws InterruptedException {
        robot.gyroscope.setTargetAngle(targetRotation);

        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);
        double totalDistance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));

        double previousDistance = 0;

        /* pid */
        double integralScaler = 0.01;
        double accelerationPercentile = 0.1;
        double accelerationBarrier = accelerationPercentile*totalDistance;
        double decelerationBarrier = totalDistance - accelerationPercentile*totalDistance;

        double speed = 0;
        double distance = totalDistance;

        double stopDistance = 5;
        while (distance > stopDistance){
            /* pid */
            double traveledDistance = totalDistance-distance;
            double previousTraveledDistance = totalDistance-previousDistance;

            /* proportional */
            if(traveledDistance <= accelerationBarrier){
                double remainingSpeedIncrease = 1-speed;

                double currentMovementPercentage = (traveledDistance - previousTraveledDistance)/(accelerationBarrier - previousTraveledDistance);
                currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                speed += remainingSpeedIncrease*currentMovementPercentage;
            }
            if(traveledDistance >= decelerationBarrier){
                double remainingSpeedDecrease = speed;

                double currentMovementPercentage = (traveledDistance - previousTraveledDistance)/(totalDistance - previousTraveledDistance);
                currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                speed -= remainingSpeedDecrease*currentMovementPercentage;
            }

            /* integral */
            if(distance == previousDistance)
                speed += integralScaler;


            /* drive towards targetPos with speed from pid */
            double angleRad = Math.toRadians(robot.gyroscope.getCurrentAngle());
            Matrix rotMatrix = new Matrix(new double[][]{
                    { Math.cos(angleRad), -Math.sin(angleRad) },
                    { Math.sin(angleRad), Math.cos(angleRad) },
            });

            Matrix relativeDeltaPosMatrix = Matrix.multiply(deltaPos.toMatrix(), rotMatrix);
            Vector2 relativeDeltaPosVector = relativeDeltaPosMatrix.toVector2();

            double angleCorrection = getWheelCorrection();
            WheelPowerConfig wpc = new WheelPowerConfig(
                    relativeDeltaPosVector.y + relativeDeltaPosVector.x + angleCorrection,
                    relativeDeltaPosVector.y - relativeDeltaPosVector.x - angleCorrection,
                    relativeDeltaPosVector.y + relativeDeltaPosVector.x - angleCorrection,
                    relativeDeltaPosVector.y - relativeDeltaPosVector.x + angleCorrection
            );
            wpc.clampScale();

            wpc = WheelPowerConfig.multiply(wpc, speed*speedScaler);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            previousDistance = distance;
            deltaPos = Vector2.subtract(targetPos, currentPosition);
            distance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));

        rotateToTargetAngle(speedScaler);
    }
    public void rotateToTargetAngle(double speedScaler) throws InterruptedException {
        double deltaAngle = MathFunctions.clambAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
        double totalAngle = Math.abs(deltaAngle);

        double previousAngle = 0;

        /* pid */
        double integralScaler = 0.01;
        double accelerationPercentile = 0.1;
        double accelerationBarrier = accelerationPercentile*totalAngle;
        double decelerationBarrier = totalAngle - accelerationPercentile*totalAngle;

        double speed = 0;
        double angle = totalAngle;

        double stopAngle = 5;
        while (angle > stopAngle){
            /* pid */
            double traveledAngle = totalAngle-angle;
            double previousTraveledAngle= totalAngle-previousAngle;

            /* proportional */
            if(traveledAngle <= accelerationBarrier){
                double remainingSpeedIncrease = 1-speed;

                double currentMovementPercentage = (traveledAngle - previousTraveledAngle)/(accelerationBarrier - previousTraveledAngle);
                currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                speed += remainingSpeedIncrease*currentMovementPercentage;
            }
            if(traveledAngle >= decelerationBarrier){
                double remainingSpeedDecrease = speed;

                double currentMovementPercentage = (traveledAngle - previousTraveledAngle)/(totalAngle - previousTraveledAngle);
                currentMovementPercentage = Math.min(currentMovementPercentage, 1); // clamp at 100%

                speed -= remainingSpeedDecrease*currentMovementPercentage;
            }

            /* integral */
            if(angle == previousAngle)
                speed += integralScaler;


            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaAngle,
                    -deltaAngle,
                    -deltaAngle,
                    deltaAngle
            );
            wpc.clampScale();

            wpc = WheelPowerConfig.multiply(wpc, speed*speedScaler);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            previousAngle = angle;
            deltaAngle = MathFunctions.clambAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
            angle = Math.abs(deltaAngle);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }

    private double getWheelCorrection(){
        double scaler = 0.03;
        double maxCorrection = 0.25;

        double targetAngle = robot.gyroscope.getTargetAngle();
        double currentAngle = robot.gyroscope.getCurrentAngle();
        double deltaAngle = targetAngle - currentAngle;

        double correction = deltaAngle*scaler;

        return Math.max(maxCorrection, correction);
    }
}
