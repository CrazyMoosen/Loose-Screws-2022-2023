package org.firstinspires.ftc.team22012.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MonishPython extends OpenCvPipeline {
    Telemetry telemetry;

    public MonishPython(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Scalar blue_lower = new Scalar(62, 62, 81);
        Scalar blue_upper = new Scalar(179, 255, 255);
        List<Double> matches;

        //image to read
        Mat refBlue = scale(input, 0.5);
        //image to compare to
        Mat blueCone = scale(Imgcodecs.imread("ConeBlue.png"), 0.5);

        //refBlue converts to HSV
        Mat hsvPic = new Mat();
        Imgproc.cvtColor(refBlue, hsvPic, Imgproc.COLOR_BGR2HSV);

        //the thresholded frame that is black and white and shows the blue in the picture.
        Mat threshold = new Mat();
        Core.inRange(hsvPic, blue_lower, blue_upper, threshold);

        Mat coneContour = approxDrawContour(threshold, refBlue, 500);
        List<MatOfPoint2f> coneRef = approxDrawRef(threshold, refBlue, 500);

        matches = isCone(blueCone, coneRef.get(0));
        System.out.println(matches);
        for (Double match : matches) {
            System.out.println(match);
        }

        return threshold;
    }

    public MatOfPoint2f matOfPoint2MatOfPoint2f(MatOfPoint matOfPoint) {
        return new MatOfPoint2f(matOfPoint.toArray());
    }

    public Mat scale(Mat input, double scale) {
        Size size = new Size();
        size.width = input.width() * scale;
        size.height = input.height() * scale;
        Mat output = new Mat();
        Imgproc.resize(input, output, size);
        return output;
    }

    public List<Double> isCone(Mat inputImage, Mat refCone) {
        List<MatOfPoint2f> objects;
        List<Double> matchedPercentages = new ArrayList<>();

        //destination image
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(inputImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        Mat threshold = new Mat();
        Core.inRange(hsvImage, new Scalar(62, 62, 81), new Scalar(179, 255, 255), threshold);

        Mat contourImg = approxDrawContour(threshold, inputImage, 500);
        objects = approxDrawRef(threshold, inputImage, 500);

        for (int i = 0; i < objects.size(); i++) {
            double match = Imgproc.matchShapes(objects.get(i), refCone, 1, 0.0);
            matchedPercentages.add(match);
        }
        return matchedPercentages;
    }

    public Mat approxDrawContour(Mat inputFrame, Mat canvas, int areaRequirement) {
        Mat img = inputFrame;
        Mat drawImage = canvas;

        //this gets changed by the findCountours() function
        List<MatOfPoint> contours = new ArrayList<>();
                                            //hierarchy variable in main.py
        Imgproc.findContours(img, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f approx = null;
        List<MatOfPoint2f> cones = new ArrayList<>();

        for (MatOfPoint cnt : contours) {
            double epsilon = 0.002 * Imgproc.arcLength(matOfPoint2MatOfPoint2f(cnt), true);
            Imgproc.approxPolyDP(matOfPoint2MatOfPoint2f(cnt), approx, epsilon, true);
            Imgproc.drawContours(drawImage, contours, contours.indexOf(cnt), new Scalar(122, 92, 0), 3);
            cones.add(approx);

        }
        return drawImage;

    }
    public List<MatOfPoint2f> approxDrawRef(Mat inputFrame, Mat canvas, int areaRequirement) {
        Mat img = inputFrame;
        Mat drawImage = canvas;

        //this gets changed by the findCountours() function
        List<MatOfPoint> contours = new ArrayList<>();
        //hierarchy variable in main.py
        Imgproc.findContours(img, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f approx = null;
        List<MatOfPoint2f> cones = new ArrayList<>();

        for (MatOfPoint cnt : contours) {
            double epsilon = 0.002 * Imgproc.arcLength(matOfPoint2MatOfPoint2f(cnt), true);
            Imgproc.approxPolyDP(matOfPoint2MatOfPoint2f(cnt), approx, epsilon, true);
            Imgproc.drawContours(drawImage, contours, contours.indexOf(cnt), new Scalar(122, 92, 0), 3);
            cones.add(approx);

        }
        return cones;
    }



}
