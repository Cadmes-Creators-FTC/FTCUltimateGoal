package org.firstinspires.ftc.teamcode.TeleOp;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import org.firstinspires.ftc.teamcode.Robot.MainRobot;

@TeleOp(name = "wobbleGoalOnly", group = "TestingTeleOps")
public class WobbleArmOnly extends LinearOpMode {
    private MainRobot robot;

    @Override
    public void runOpMode () throws InterruptedException{
        String[] enabledComponents = {"logging", "wobbleArm"};
        robot = new MainRobot(hardwareMap, telemetry, enabledComponents,this);

        robot.logging.setLog("state", "Initializing");

        robot.startThreads();

        robot.logging.setLog("state", "Initialized, waiting for start");
        waitForStart();

        robot.logging.setLog("state", "Running");
        controlLoop();

        robot.stopRobot();
        robot.logging.setLog("state", "Stopped");
    }
    public void controlLoop(){
        while (opModeIsActive()){
            wobbleArm();
        }
    }

    private void wobbleArm(){
//        if (gamepad2.a)
//            robot.wobbleArm.armDown();
//        else if(gamepad2.b)
//            robot.wobbleArm.armUp();

        if(gamepad2.x)
            robot.wobbleArm.closeGripper();
        else if(gamepad2.y)
            robot.wobbleArm.openGripper();
    }
}
