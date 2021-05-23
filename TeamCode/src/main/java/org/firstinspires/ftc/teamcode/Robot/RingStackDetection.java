package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class RingStackDetection extends RobotComponent {
    private OpenCvInternalCamera phoneCam;
    private RingStackDetermenationPipeline camPipeline;

    public RingStackDetection(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        camPipeline = new RingStackDetermenationPipeline();
        phoneCam.setPipeline(camPipeline);
        phoneCam.startStreaming(176, 144, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    @Override
    public void startThreads(){

    }

    public int getStackSize(){
        int avgRedVal = camPipeline.getAvgRedVal();

        int oneRingThreshhold = 123;
        int fourRingThreshhold = 130;

        int stackSize = 0;
        if(avgRedVal > fourRingThreshhold)
            stackSize = 4;
        else if(avgRedVal > oneRingThreshhold)
            stackSize = 1;

        return stackSize;
    }
    public int getRedVal(){
        return camPipeline.getAvgRedVal();
    }

    private static class RingStackDetermenationPipeline extends OpenCvPipeline{
        static final Point REGION1_TOPLEFT = new Point(0,105);
        static final int REGION1_WIDTH = 45;
        static final int REGION1_HEIGHT = 33;
        static final Point region1Start = new Point(
                REGION1_TOPLEFT.x,
                REGION1_TOPLEFT.y
        );
        static final Point region1End = new Point(
                REGION1_TOPLEFT.x + REGION1_WIDTH,
                REGION1_TOPLEFT.y + REGION1_HEIGHT
        );


        Mat workingMatrix = new Mat();
        int avgRedVal;

        @Override
        public Mat processFrame(Mat input)
        {
            input.copyTo(workingMatrix);

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(workingMatrix, workingMatrix, 1);

            Mat subMatRegion1 = workingMatrix.submat(new Rect(region1Start, region1End));
            avgRedVal = (int) Core.mean(subMatRegion1).val[0];

            //add rectangles to show target zone
            Imgproc.rectangle(
                    input,
                    region1Start,
                    region1End,
                    new Scalar(255, 0, 0),
                    1
            );

            return input;
        }
        
        public int getAvgRedVal(){
            return avgRedVal;
        }
    }
}
