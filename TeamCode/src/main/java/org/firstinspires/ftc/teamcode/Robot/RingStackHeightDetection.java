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
public class RingStackHeightDetection extends RobotComponent {
    private OpenCvInternalCamera phoneCam;
    private RingStackDetermenationPipeline camPipeline;

    public RingStackHeightDetection(HardwareMap hardwareMap, MainRobot inputRobot) {
        super(inputRobot);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        camPipeline = new RingStackDetermenationPipeline();
        phoneCam.setPipeline(camPipeline);
        phoneCam.startStreaming(176, 144, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void startThreads(){

    }

    public int getStackSize(){
        int avgRedVal = camPipeline.getAvgRedVal();

        int oneRingThreshhold = 126;
        int fourRingThreshhold = 131;

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
        static final Point REGION1_TOPLEFT = new Point(30,126);
        static final int REGION1_WIDTH = 35;
        static final int REGION1_HEIGHT = 30;
        static final Point region1Start = new Point(
                REGION1_TOPLEFT.x,
                REGION1_TOPLEFT.y
        );
        static final Point region1End = new Point(
                REGION1_TOPLEFT.x + REGION1_WIDTH,
                REGION1_TOPLEFT.y + REGION1_HEIGHT
        );

        static final Point REGION2_TOPLEFT = new Point(101,126);
        static final int REGION2_WIDTH = 35;
        static final int REGION2_HEIGHT = 30;
        static final Point region2Start = new Point(
                REGION2_TOPLEFT.x,
                REGION2_TOPLEFT.y
        );
        static final Point region2End = new Point(
                REGION2_TOPLEFT.x + REGION2_WIDTH,
                REGION2_TOPLEFT.y + REGION2_HEIGHT
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
            int avgRedValRegion1 = (int) Core.mean(subMatRegion1).val[0];

            Mat subMatRegion2 = workingMatrix.submat(new Rect(region2Start, region2End));
            int avgRedValRegion2 = (int) Core.mean(subMatRegion2).val[0];

            avgRedVal = (avgRedValRegion1 + avgRedValRegion2)/2;

            //add rectangles to show target zone
            Imgproc.rectangle(
                    input,
                    region1Start,
                    region1End,
                    new Scalar(255, 0, 0),
                    1
            );
            Imgproc.rectangle(
                    input,
                    region2Start,
                    region2End,
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
