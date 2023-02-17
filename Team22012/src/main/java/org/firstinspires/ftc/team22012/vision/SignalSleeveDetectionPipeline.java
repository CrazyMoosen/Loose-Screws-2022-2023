package org.firstinspires.ftc.team22012.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleeveDetectionPipeline extends OpenCvPipeline {
    public enum SignalSleeveColor {
        GREEN,
        YELLOW,
        PURPLE,
        NONE;
    }

    private final Rect ROIRect = new Rect(new Point(250, 100), new Point(500, 410));

    private SignalSleeveColor sleeveColor = SignalSleeveColor.NONE;

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat roiHSV = hsv.submat(ROIRect);

        Scalar greenLower = new Scalar(34, 99, 60);
        Scalar greenUpper = new Scalar(69, 191, 239);

        Scalar yellowLower = new Scalar(20, 75, 120);
        Scalar yellowUpper = new Scalar(32, 255, 255);

        Scalar purpleLower = new Scalar(160, 20, 80);
        Scalar purpleUpper = new Scalar(179, 170, 255);

        Mat greenThresh = new Mat();
        Core.inRange(roiHSV, greenLower, greenUpper, greenThresh);
        Mat yellowThresh = new Mat();
        Core.inRange(roiHSV, yellowLower, yellowUpper, yellowThresh);
        Mat purpleThresh = new Mat();
        Core.inRange(roiHSV, purpleLower, purpleUpper, purpleThresh);

        double greenRoiPercent = Core.sumElems(greenThresh).val[0] / ROIRect.area() / 255;
        double yellowRoiPercent = Core.sumElems(yellowThresh).val[0] / ROIRect.area() / 255;
        double purpleRoiPercent = Core.sumElems(purpleThresh).val[0] / ROIRect.area() / 255;

        if (greenRoiPercent > yellowRoiPercent && greenRoiPercent > purpleRoiPercent && greenRoiPercent > 0.2) {
            sleeveColor = SignalSleeveColor.GREEN;
        }
        else if (yellowRoiPercent > purpleRoiPercent && yellowRoiPercent > greenRoiPercent && yellowRoiPercent > 0.2) {
            sleeveColor = SignalSleeveColor.YELLOW;
        }
        else if (purpleRoiPercent > yellowRoiPercent && purpleRoiPercent > greenRoiPercent && purpleRoiPercent > 0.2) {
            sleeveColor = SignalSleeveColor.PURPLE;
        }
        else {
            sleeveColor = SignalSleeveColor.NONE;
        }

        Mat outline = new Mat();
        Core.bitwise_and(input, input, outline, greenThresh);
        greenThresh.release();
        yellowThresh.release();
        purpleThresh.release();

        Imgproc.rectangle(outline, ROIRect, new Scalar(255, 0, 0), 3);

        return outline;
    }

    public SignalSleeveColor getSleeveColor() {
        return sleeveColor;
    }

}
