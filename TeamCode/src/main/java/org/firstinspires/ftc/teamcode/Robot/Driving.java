package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Matrix;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPosition;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;

@Disabled
public class Driving extends RobotComponent {
    private final DcMotor wheelLF;
    private final DcMotor wheelRF;
    private final DcMotor wheelRB;
    private final DcMotor wheelLB;
    private final int ticksPerRotation = 960;

    private Vector2 currentPosition = new Vector2(0, 0);
    private WheelPosition currentWheelPosTicks = new WheelPosition(0, 0, 0, 0);

    public Driving(HardwareMap hardwareMap, Telemetry inputTelemetry, MainRobot inputRobot) {
        super(inputTelemetry, inputRobot);

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
    }

    @Override
    public void startThreats(){
        new Thread(){
            @Override
            public void run(){
                try {
                    keepPositionUpdated();
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

    //region Position
    public Vector2 getCurrentPosition(){
        return currentPosition;
    }
    public void setCurrentPosition(Vector2 pos){
        currentPosition = pos;
    }

    public void keepPositionUpdated() throws InterruptedException{
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

            /* get wheel pos matrix */
            wheelPosDelta.toCM(11*Math.PI, ticksPerRotation);
            Matrix wheelPosMatrix = new Matrix(1, 4, new double[][]{
                    { wheelPosDelta.lf, wheelPosDelta.rf, wheelPosDelta.lb, wheelPosDelta.rb }
            });

            /* get transformation matrix */
            double angle = Math.toRadians(-robot.gyroscope.getCurrentAngle()) + Math.PI/4;
            double sinVal = Math.sqrt(2)*Math.sin(angle);
            double cosVal = Math.sqrt(2)*Math.cos(angle);
            Matrix transformMatrix = new Matrix(4, 2, new double[][]{
                    {  cosVal, 1.21*sinVal },
                    { -sinVal, 1.21*cosVal },
                    { -sinVal, 1.21*cosVal },
                    {  cosVal, 1.21*sinVal },
            });
            transformMatrix = Matrix.scale(transformMatrix, 0.25);

            /* get movement */
            Matrix deltaPosMatrix = Matrix.multiply(wheelPosMatrix, transformMatrix);
            Vector2 deltaPosVector = deltaPosMatrix.toVector2();

            /* update position */
            currentPosition = Vector2.add(currentPosition, deltaPosVector);

            /* timeout between updates */
            Thread.sleep(50);
        }
    }
    public void driveToPosition(Vector2 targetPos) throws InterruptedException {
        double stopDistance = 10;

        Vector2 deltaPos = Vector2.subtract(targetPos, currentPosition);
        while ((Math.abs(deltaPos.x) > stopDistance || Math.abs(deltaPos.y) > stopDistance)){

            double angleRad = Math.toRadians(robot.gyroscope.getCurrentAngle());
            Matrix rotMatrix = new Matrix(2, 2, new double[][]{
                    { Math.cos(angleRad), -Math.sin(angleRad) },
                    { Math.sin(angleRad), Math.cos(angleRad) },
            });

            Matrix relativeDeltaPosMatrix = Matrix.multiply(deltaPos.toMatrix(), rotMatrix);
            Vector2 relativeDeltaPosVector = relativeDeltaPosMatrix.toVector2();

            WheelPowerConfig wpc = new WheelPowerConfig(
                    relativeDeltaPosVector.y + relativeDeltaPosVector.x,
                    relativeDeltaPosVector.y - relativeDeltaPosVector.x,
                    relativeDeltaPosVector.y + relativeDeltaPosVector.x,
                    relativeDeltaPosVector.y - relativeDeltaPosVector.x
            );

            wpc.clamp();
            setWheelPowers(wpc);

            Thread.sleep(100);

            deltaPos = Vector2.subtract(targetPos, currentPosition);
        };

        setWheelPowers(new WheelPowerConfig(0, 0, 0, 0));
    }

    //endregion
}
