package org.firstinspires.ftc.team22012.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat matrix = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        BOTH,
        NOT_FOUND;
    }

    private Location location;

    /**
    These 2 are the regions of interest the camera will look in
    to find our cones, by checking to see what percentage of the
    threshold is white
    **/
    static final Rect LEFT_ROI = new Rect(
            new Point(30, 35),
            new Point(120, 205)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(230, 205)
    );

    //threshold value that identifies the cone based on how much blue it has
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public ConeDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //input matrix is a video frame we get from the camera
    @Override
    public Mat processFrame(Mat input) {
        //converts RGB values on frame to HSV values
        Imgproc.cvtColor(input, matrix, Imgproc.COLOR_RGB2HSV);

        //anything in these 2 range of HSV values are considered 'blue'
        Scalar minHSV = new Scalar(200, 100, 100);
        Scalar maxHSV = new Scalar(257, 100, 100);

        Core.inRange(matrix, minHSV, maxHSV, matrix);

        Mat left = matrix.submat(LEFT_ROI);
        Mat right = matrix.submat(RIGHT_ROI);

        //percent blue in the left and right region of interest
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        //Debugging, since something will def go wrong xd
        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean coneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean coneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (coneLeft && coneRight) {
            location = Location.BOTH;
            telemetry.addData("Cone Location", "both");
        }
        else if (coneLeft) {
            location = Location.LEFT;
            telemetry.addData("Cone Location", "left");
        }
        else if (coneRight) {
            location = Location.RIGHT;
            telemetry.addData("Cone Location", "right");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cone Location", "not found");
        }
        telemetry.update();

        //convert the grayscale image back to RGB so we can draw rectangles over the cones
        Imgproc.cvtColor(matrix, matrix, Imgproc.COLOR_GRAY2RGB);

        Scalar colorCone = new Scalar(0, 0, 255);
        Scalar colorNotCone = new Scalar(255, 0, 0);

        //draw the rectangles and if there's a cone, make it blue else make it red
        Imgproc.rectangle(matrix, LEFT_ROI, location == Location.LEFT ? colorCone:colorNotCone);
        Imgproc.rectangle(matrix, RIGHT_ROI, location == Location.RIGHT ? colorCone:colorNotCone);

        return matrix;
    }

    public Location getLocation() {
        return location;
    }
}
