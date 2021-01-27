package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class RingStackHeightDetection extends RobotComponent {
    OpenCvInternalCamera phoneCam;
    private final RingStackDetermenationPipeline camPipeline;

    public RingStackHeightDetection(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        camPipeline = new RingStackDetermenationPipeline();
        phoneCam.setPipeline(camPipeline);
    }

    @Override
    public void startThreads(){

    }
}
