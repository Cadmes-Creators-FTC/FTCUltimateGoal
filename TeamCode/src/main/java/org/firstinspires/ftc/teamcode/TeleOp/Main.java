package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.DataTypes.Vector2;
import org.firstinspires.ftc.teamcode.Misc.MathFunctions;
import org.firstinspires.ftc.teamcode.Robot.MainRobot;
import org.firstinspires.ftc.teamcode.Misc.DataTypes.WheelPowerConfig;

@TeleOp(name = "main", group = "GameTeleOps")
public class Main extends LinearOpMode {
    private MainRobot robot;

    double drivingDirection = 1;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "gyroscope", "driving", "shooter", "wobbleArm", "intake", "conveyor"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents, this);

        robot.logging.setLog("state", "Initializing");
        robot.gyroscope.waitForGyroCalibration();
        robot.driving.resetWheelTicks();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.logging.clearLogs();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop() throws InterruptedException{
        while (opModeIsActive()){
            drive();
            setDrivingDirection();
            driveToShootingPos();

            intake();
            conveyor();
            shooter();

            wobbleArm();

            robot.logging.setLog("average wheel speed", robot.driving.getWheelPowers().getAverage());
            robot.logging.setLog("drive-direction", drivingDirection == 1 ? "forward" : "reverse");
            robot.logging.setLog("pos", robot.driving.getCurrentPosition());
            robot.logging.setLog("rot", robot.gyroscope.getCurrentAngle());
        }
    }


    boolean intakeStateChanged = false;
    private void intake(){
        if (gamepad2.b && !intakeStateChanged){
            intakeStateChanged = true;
            if (robot.intake.isOn())
                robot.intake.turnOff();
            else
                robot.intake.turnOn();
        }else if(!gamepad2.b)
            intakeStateChanged = false;
    }

    boolean conveyorStateChanged = false;
    boolean conveyorTriggerState = false;
    private void conveyor(){
        if (gamepad2.left_bumper && !conveyorStateChanged){
            conveyorStateChanged = true;
            if (robot.conveyor.isOn())
                robot.conveyor.turnOff();
            else
                robot.conveyor.turnOn(0.3);
        } else if(!gamepad2.left_bumper)
            conveyorStateChanged = false;

            if(gamepad2.left_trigger > 0.5){
                conveyorTriggerState = true;
                robot.conveyor.turnOn(1);
            }else if (gamepad2.left_trigger < 0.5 && conveyorTriggerState){
                conveyorTriggerState = false;
                robot.conveyor.turnOff();
            }
    }

    boolean shooterStateChanged = false;
    private void shooter(){
        if (gamepad2.a && !shooterStateChanged){
            shooterStateChanged = true;
            if (robot.shooter.isOn())
                robot.shooter.turnOff();
            else
                robot.shooter.turnOn(0.97);
        }
        if(!gamepad2.a)
            shooterStateChanged = false;
    }
    
    boolean wobbleArmGripperStateChanged = false;
    private void wobbleArm(){
        if (gamepad2.right_trigger > 0){
            robot.wobbleArm.armDown();
        } else if (gamepad2.right_bumper){
            robot.wobbleArm.armUp();
        }

        if (gamepad2.y && !wobbleArmGripperStateChanged){
            wobbleArmGripperStateChanged = true;
            if (robot.wobbleArm.isGripperOpen())
                robot.wobbleArm.closeGripper();
            else
                robot.wobbleArm.openGripper();
        }
        if(!gamepad2.y)
            wobbleArmGripperStateChanged = false;
    }

    private void drive(){
        //get joystick input
        double forwardInput = gamepad1.left_stick_y*-1;//reversing joystick so up becomes forward
        double strafeInput = gamepad1.right_stick_x;
        double rotationInput = gamepad1.right_trigger - gamepad1.left_trigger;

        forwardInput *= drivingDirection;
        strafeInput *= drivingDirection;

        double joyMinInput = 0.05;
        double triggerMinInput = 0.05;

        //scale 0.2-1 to 0-1
        forwardInput = MathFunctions.absXOverX(forwardInput) * Math.max(0, (Math.abs(forwardInput)-joyMinInput) / (1-joyMinInput));
        strafeInput = MathFunctions.absXOverX(strafeInput) * Math.max(0, (Math.abs(strafeInput)-joyMinInput) / (1-joyMinInput));
        rotationInput = MathFunctions.absXOverX(rotationInput) * Math.max(0, (Math.abs(rotationInput)-triggerMinInput) / (1-triggerMinInput));

        //smooth out the values - f(x) = curveVal*x^2 + (1-curveVal)*x
        double curveVal = 0.3;
        if(forwardInput != 0)
            forwardInput = curveVal*(Math.pow(forwardInput, 2)*MathFunctions.absXOverX(forwardInput)) + (1-curveVal)*forwardInput;
        if(strafeInput != 0)
            strafeInput = curveVal*(Math.pow(strafeInput, 2)*MathFunctions.absXOverX(strafeInput)) + (1-curveVal)*strafeInput;
        if(rotationInput != 0)
            rotationInput = curveVal*(Math.pow(rotationInput, 2)*MathFunctions.absXOverX(rotationInput)) + (1-curveVal)*rotationInput;

        //create wheel power config
        WheelPowerConfig wpc = new WheelPowerConfig(
                forwardInput + strafeInput + rotationInput,
                forwardInput - strafeInput - rotationInput,
                forwardInput + strafeInput - rotationInput,
                forwardInput - strafeInput + rotationInput
        );
        wpc.clamp();

        robot.driving.setWheelPowers(wpc);
    }
    private void driveToShootingPos() throws InterruptedException{
        if(gamepad1.b){
            robot.driving.driveToPosition(new Vector2(115, 160), -180.0, 0.5);
            robot.logging.removeLog("dist");
            robot.logging.removeLog("distTotal");
            robot.logging.removeLog("accelerationBarrier");
            robot.logging.removeLog("pos");
            robot.logging.removeLog("speed");
            robot.logging.removeLog("speedAfterScale");
        }
    }
    boolean drivingDirectionStateChanged = false;
    private void setDrivingDirection(){
        if (gamepad1.a && !drivingDirectionStateChanged){
            drivingDirectionStateChanged = true;

            drivingDirection *= -1;
        }
        if(!gamepad1.a)
            drivingDirectionStateChanged = false;
    }
}