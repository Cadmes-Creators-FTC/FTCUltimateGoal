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
            double yScaler = 1.6;
            double xScaler = 1.2;

            double deltaX = ((wheelPosDelta.lf + wheelPosDelta.rb) - (wheelPosDelta.rf + wheelPosDelta.lb)) / 4;
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
            Thread.sleep(10);
        }
    }


    public void driveToPosition(Vector2 targetPos, Double targetAngle, double speedScaler) throws InterruptedException {
        if(targetAngle != null)
            robot.gyroscope.setTargetAngle(targetAngle);

        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);
        double totalDistance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));
        double distance = totalDistance;

        /* function config */
        //acceleration
        double accelerationTime = 1;
        //deceleration
        double maxSpeedDecelerationDistance = 50;
        //min speed
        double minSpeedY = 0.15;
        double minSpeedX = 0.3;
        //rotation
        double maxAngleCorrection = 0.5;
        //stopping
        double stopDistance = 5;

        long startTime = System.currentTimeMillis();// stap 1
        double decelerationDistance = maxSpeedDecelerationDistance*speedScaler;

        while (robot.isRunning && (distance > stopDistance)){
            double speed;
            if(distance < decelerationDistance){ //decelerate
                speed = distance/decelerationDistance;
            }else{ // accelerate and keep speed
                long time = System.currentTimeMillis() - startTime; //stap 2
                speed = Math.min(1, time/(accelerationTime*1000));// stap 3/4
            }

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

            double deltaAngle = (Math.toDegrees(Math.atan2(deltaPos.y, deltaPos.x))*-1)-90;
            double minSpeedForMovement = minSpeedY + (minSpeedX-minSpeedY)*(-(1.0/90.0)*Math.abs(Math.abs(deltaAngle)-90)+1);
            double scaledSpeed = ((speed*(1-minSpeedForMovement))+minSpeedForMovement)*speedScaler;
            wpc = WheelPowerConfig.multiply(wpc, scaledSpeed);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            deltaPos = Vector2.subtract(targetPos, currentPosition);
            distance = Math.sqrt(Math.pow(deltaPos.x, 2) + Math.pow(deltaPos.y, 2));

            robot.logging.setLog("pos", getCurrentPosition());
            robot.logging.setLog("dPos", deltaPos);
            robot.logging.setLog("scaledSpeed", scaledSpeed);
            robot.logging.setLog("minSpeed", minSpeedForMovement);
        }

        if(targetAngle != null)
            rotateToAngle(targetAngle, speedScaler);

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    public void driveToPositionPID(Vector2 targetPos, double speedScaler) throws InterruptedException{
        Vector2 curPos = getCurrentPosition();
        Vector2 deltaPos = Vector2.subtract(targetPos, curPos);
        double error = Math.sqrt(Math.pow(deltaPos.x, 2)+Math.pow(deltaPos.y, 2));

        double kP = 0;
        double kI = 0;
        double iLimit = 0;
        double kD = 0;

        double errorSum = 0;
        double lastTimeStamp = (((double)System.currentTimeMillis())/1000);
        double lastError = error;

        double maxAngleCorrection = 0.5;

        double preferredAngle = 0;

        double stopError = 5;
//        while (robot.isRunning && (error > stopError)) {
        while (robot.isRunning) {
            double dt = (((double)System.currentTimeMillis())/1000) - lastTimeStamp;
            lastTimeStamp = (((double)System.currentTimeMillis())/1000); //do this as close as possible to dt to minimize time loss

            if(error < iLimit)
                errorSum += dt*error;

            double errorRate = (error-lastError)/dt;

            double speed = kP*error + kI*errorSum + kD*errorRate;


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

            double absAngleToTarget = Math.atan2(deltaPos.x, deltaPos.y);
            robot.gyroscope.setTargetAngle(MathFunctions.clampAngleDegrees(absAngleToTarget+preferredAngle));
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

            wpc = WheelPowerConfig.multiply(wpc, speed*speedScaler);
            setWheelPowers(wpc);


            Thread.sleep(20);


            curPos = getCurrentPosition();
            deltaPos = Vector2.subtract(targetPos, curPos);
            lastError = error;
            error = Math.sqrt(Math.pow(deltaPos.x, 2)+Math.pow(deltaPos.y, 2));
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }
    public void rotateToAngle(double targetAngle, double speedScaler) throws InterruptedException {
        robot.gyroscope.setTargetAngle(targetAngle);

        double deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
        double totalAngle = Math.abs(deltaAngle);
        double angle = totalAngle;

        /* function config */
        //acceleration
        double accelerationTime = 0.25;
        //deceleration
        double maxSpeedDecelerationDistance = 45;
        //min speed
        double minSpeedForMovement = 0.15;
        //stopping
        double stopAngle = 4;

        long startTime = System.currentTimeMillis();
        double decelerationDistance = maxSpeedDecelerationDistance*speedScaler;

        while (robot.isRunning && (angle > stopAngle)){
            double speed;
            if(angle < decelerationDistance){ //decelerate
                speed = angle/decelerationDistance;
            }else{ // accelerate and keep speed
                long time = System.currentTimeMillis() - startTime; //stap 2
                speed = Math.min(1, time/(accelerationTime*1000));// stap 3/4
            }

            WheelPowerConfig wpc = new WheelPowerConfig(
                    deltaAngle,
                    -deltaAngle,
                    -deltaAngle,
                    deltaAngle
            );
            wpc.clampScale();

            double scaledSpeed = ((speed*(1-minSpeedForMovement))+minSpeedForMovement)*speedScaler;
            wpc = WheelPowerConfig.multiply(wpc, scaledSpeed);
            setWheelPowers(wpc);


            Thread.sleep(50);

            /* set values for next loop run */
            deltaAngle = MathFunctions.clampAngleDegrees(robot.gyroscope.getTargetAngle() - robot.gyroscope.getCurrentAngle());
            angle = Math.abs(deltaAngle);
        }

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }


    //              |
    //kut functies  |
    //             \|/
    public void driveToPositionYOnly(Vector2 targetPos, Double targetAngle, double speedScaler, Double driveDirection) throws InterruptedException{
        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);
        double angleForward = (Math.toDegrees(Math.atan2(deltaPos.y, deltaPos.x)) - 90) * -1;
        double angleBackward = MathFunctions.clampAngleDegrees(angleForward+180);

        if(driveDirection == null) {
            double deltaAngleForward = Math.abs(angleForward - robot.gyroscope.getCurrentAngle());
            double deltaAngleBackward = Math.abs(angleBackward - robot.gyroscope.getCurrentAngle());
            if (targetAngle != null) {
                deltaAngleForward += Math.abs(angleForward - targetAngle);
                deltaAngleBackward += Math.abs(angleBackward - targetAngle);
            }
            if (deltaAngleForward <= deltaAngleBackward)
                rotateToAngle(angleForward, speedScaler);
            else
                rotateToAngle(angleBackward, speedScaler);
        }else if(driveDirection == 1)
            rotateToAngle(angleForward, 1);
        else if(driveDirection == -1)
            rotateToAngle(angleBackward, 1);

        driveToPosition(targetPos, null, speedScaler);

        if(targetAngle != null)
            robot.driving.rotateToAngle(targetAngle, speedScaler);
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
    //             /|\
    //kut functies  |
    //              |


    private double getAngleCorrection(){
        double scaler = 0.02;

        double targetAngle = robot.gyroscope.getTargetAngle();
        double currentAngle = robot.gyroscope.getCurrentAngle();
        double deltaAngle = targetAngle - currentAngle;

        return deltaAngle*scaler;
    }
}
