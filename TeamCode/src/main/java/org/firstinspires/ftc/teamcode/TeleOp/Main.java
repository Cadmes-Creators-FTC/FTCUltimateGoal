package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents);

        robot.logging.setLog("state", "Initializing");
        robot.gyroscope.waitForGyroCalibration();
        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.logging.clearLogs();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }

    private void controlLoop(){
        while (opModeIsActive()){
            drive();
//            slowMovement();
            setDrivingDirection();

            intake();
            conveyor();
            shooter();

            wobbleArm();

            robot.logging.setLog("average wheel speed", robot.driving.getWheelPowers().getAverage());
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
                robot.shooter.turnOn(0.95);
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

    int enabledDriveControls = 0;
    private void driveOld(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        enabledDriveControls = (joyX != 0 || joyY != 0 || joyR != 0) ? 0 : enabledDriveControls;
        if(enabledDriveControls != 0)
            return;

        //smooth out the joysticks - f(x) = 0.6x^2 + 0.4x
//        if(joyX != 0)
//            joyX = 0.6*(Math.pow(joyX, 2)*(joyX/Math.abs(joyX)))+0.4*joyX;
//        if(joyY != 0)
//            joyY = 0.6*(Math.pow(joyY, 2)*(joyY/Math.abs(joyY)))+0.4*joyY;
//        if(joyR != 0)
//            joyR = 0.6*(Math.pow(joyR, 2)*(joyR/Math.abs(joyR)))+0.4*joyR;

        joyX *= drivingDirection;
        joyY *= drivingDirection;

        //reverse y joystick
        joyY *= -1;

        //create wheel power config
        WheelPowerConfig wpc = new WheelPowerConfig(
                joyY + joyX + joyR,
                joyY - joyX - joyR,
                joyY + joyX - joyR,
                joyY - joyX + joyR
        );
        wpc.clamp();

        robot.driving.setWheelPowers(wpc);
    }
    private void drive(){
        //get joystick input
        double forwardInput = gamepad1.left_stick_y*-1;//reversing joystick so up becomes forward
        double strafeInput = gamepad1.right_stick_x;
        double rotationInput = gamepad1.right_trigger - gamepad1.left_trigger;

//        enabledDriveControls = (forwardInput != 0 || strafeInput != 0 || rotationInput != 0) ? 0 : enabledDriveControls;
//        if(enabledDriveControls != 0)
//            return;

        forwardInput *= drivingDirection;
        strafeInput *= drivingDirection;

        double joyMinInput = 0.2;
        double triggerMinInput = 0.2;

        //scale 0.2-1 to 0-1
        robot.logging.setLog("f-1", forwardInput);
        robot.logging.setLog("s-1", strafeInput);
        robot.logging.setLog("r-1", rotationInput);
        forwardInput = MathFunctions.absXOverX(forwardInput) * Math.max(0, (Math.abs(forwardInput)-joyMinInput) / (1-joyMinInput));
        strafeInput = MathFunctions.absXOverX(strafeInput) * Math.max(0, (Math.abs(strafeInput)-joyMinInput) / (1-joyMinInput));
        rotationInput = MathFunctions.absXOverX(rotationInput) * Math.max(0, (Math.abs(rotationInput)-triggerMinInput) / (1-triggerMinInput));
        robot.logging.setLog("f-2", forwardInput);
        robot.logging.setLog("s-2", strafeInput);
        robot.logging.setLog("r-2", rotationInput);

        //smooth out the values - f(x) = curveVal*x^2 + (1-curveVal)*x
        double curveVal = 0.3;
        if(forwardInput != 0)
            forwardInput = curveVal*(Math.pow(forwardInput, 2)*MathFunctions.absXOverX(forwardInput)) + (1-curveVal)*forwardInput;
        if(strafeInput != 0)
            strafeInput = curveVal*(Math.pow(strafeInput, 2)*MathFunctions.absXOverX(strafeInput)) + (1-curveVal)*strafeInput;
        if(rotationInput != 0)
            rotationInput = curveVal*(Math.pow(rotationInput, 2)*MathFunctions.absXOverX(rotationInput)) + (1-curveVal)*rotationInput;
        robot.logging.setLog("f-3", forwardInput);
        robot.logging.setLog("s-3", strafeInput);
        robot.logging.setLog("r-3", rotationInput);

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
    private void slowMovement(){
        enabledDriveControls = (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.left_bumper || gamepad2.right_bumper) ? 1 : enabledDriveControls;
        if(enabledDriveControls != 1)
            return;

        double power = 0.3;

        if (gamepad1.dpad_up) {
            power *= drivingDirection;
            WheelPowerConfig wpc = new WheelPowerConfig(power, power, power, power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_down) {
            power *= drivingDirection;
            WheelPowerConfig wpc = new WheelPowerConfig(-power, -power, -power, -power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_left) {
            power *= drivingDirection;
            WheelPowerConfig wpc = new WheelPowerConfig(-1.5*power, 1.5*power, -1.5*power, 1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.dpad_right){
            power *= drivingDirection;
            WheelPowerConfig wpc = new WheelPowerConfig(1.5*power, -1.5*power, 1.5*power, -1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.left_bumper) {
            WheelPowerConfig wpc = new WheelPowerConfig(-1.5*power, 1.5*power, 1.5*power, -1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else if (gamepad1.right_bumper) {
            WheelPowerConfig wpc = new WheelPowerConfig(1.5*power, -1.5*power, -1.5*power, 1.5*power);
            robot.driving.setWheelPowers(wpc);
        }else {
            WheelPowerConfig wpc = new WheelPowerConfig(0, 0, 0, 0);
            robot.driving.setWheelPowers(wpc);
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