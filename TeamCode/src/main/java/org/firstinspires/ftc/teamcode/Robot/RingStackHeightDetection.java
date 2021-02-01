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
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camPipeline = new RingStackDetermenationPipeline();
        phoneCam.setPipeline(camPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void startThreads(){

    }

    public int getStackSize(){
        int avgRedVal = camPipeline.getAvgRedVal();

        int oneRingThreshhold = 130;
        int fourRingThreshhold = 150;

        int stackSize = 0;
        if(avgRedVal > fourRingThreshhold)
            stackSize = 4;
        else if(avgRedVal > oneRingThreshhold)
            stackSize = 1;

        return stackSize;
    }

    private static class RingStackDetermenationPipeline extends OpenCvPipeline{
        static final Point REGION_TOPLEFT = new Point(181,98);
        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        Point targetRegionStart = new Point(
                REGION_TOPLEFT.x,
                REGION_TOPLEFT.y
        );
        Point targetRegionEnd = new Point(
                REGION_TOPLEFT.x + REGION_WIDTH,
                REGION_TOPLEFT.y + REGION_HEIGHT
        );

        Mat workingMatrix = new Mat();
        int avgRedVal;

        @Override
        public Mat processFrame(Mat input)
        {
            input.copyTo(workingMatrix);

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(workingMatrix, workingMatrix, 1);

            Mat subMat = workingMatrix.submat(new Rect(targetRegionStart, targetRegionEnd));

            avgRedVal = (int) Core.mean(subMat).val[0];

            //display rectangle?

            return workingMatrix;

//            Imgproc.rectangle(
//                    input,
//                    targetRegionStart,
//                    targetRegionEnd,
//                    BLUE,
//                    2
//            );
        }

        public int getAvgRedVal(){
            return avgRedVal;
        }
    }
}
