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


    boolean prevIntakeBtn = false;
    private void intake(){
        boolean intakeBtn = gamepad2.b;
        if(intakeBtn && !prevIntakeBtn){
            if(robot.intake.isOn())
                robot.intake.turnOff();
            else
                robot.intake.turnOn();
        }
        prevIntakeBtn = intakeBtn;
    }

    boolean prevConveyorBtn = false;
    private void conveyor(){
        boolean conveyorBtn = gamepad2.left_bumper;
        boolean conveyorTriggerBtn = gamepad2.left_trigger > 0.5;

        if(conveyorBtn && !prevConveyorBtn){
            if(robot.conveyor.isOn())
                robot.conveyor.turnOff();
            else
                robot.conveyor.turnOn(0.3);
        }else{
            if(conveyorTriggerBtn)
                robot.conveyor.turnOn(0.9);
            else if(robot.conveyor.getPower() == 0.9)
                robot.conveyor.turnOff();
        }

        prevConveyorBtn = conveyorBtn;
    }

    boolean prevShooterBtn = false;
    private void shooter(){
        boolean shooterBtn = gamepad2.a;
        if(shooterBtn && !prevShooterBtn){
            if(robot.shooter.isOn())
                robot.shooter.turnOff();
            else
                robot.shooter.turnOn(0.92);
        }
        prevShooterBtn = shooterBtn;
    }
    
    boolean prevWobbleArmGripperBtn = false;
    private void wobbleArm(){
        if (gamepad2.right_trigger > 0){
            robot.wobbleArm.armDown();
        } else if (gamepad2.right_bumper){
            robot.wobbleArm.armUp();
        }

        boolean wobbleArmGripperBtn = gamepad2.y;
        if(wobbleArmGripperBtn && !prevWobbleArmGripperBtn){
            if(robot.wobbleArm.isGripperOpen())
                robot.wobbleArm.closeGripper();
            else
                robot.wobbleArm.openGripper();
        }
        prevWobbleArmGripperBtn = wobbleArmGripperBtn;
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
    boolean prevDrivingDirectionBtn = false;
    private void setDrivingDirection(){
        boolean drivingDirectionBtn = gamepad1.y;
        if(drivingDirectionBtn && !prevDrivingDirectionBtn){
            drivingDirection *= -1;
        }
        prevDrivingDirectionBtn = drivingDirectionBtn;
    }
}