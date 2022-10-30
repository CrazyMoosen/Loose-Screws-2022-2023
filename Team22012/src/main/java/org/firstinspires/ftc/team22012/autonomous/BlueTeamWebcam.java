package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import android.graphics.Bitmap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team22012.vision.VuforiaLocalizerImplSubclass;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Vuforia Webcam Detection Blue Team")
public class BlueTeamWebcam extends LinearOpMode{
    public enum SignalSleeveLocation{
        GREEN,
        YELLOW,
        PURPLE,
        NONE;
    }
    private SignalSleeveLocation location;
    double percent = 0;
    Motor fL, fR, bL, bR;
    ElapsedTime elapsedTime = new ElapsedTime();

    // Experimentally determined variables
    double speed = 50.5; // In inches/sec
    double stoppingDistance = 1; // In inches, the distance it takes to stop the robot travelling
    // at the power of 0.8
    double degPerSec = 160;
    /**
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    int run = 0;
    boolean moved = false;
    /**
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaLoWT//////AAABmWaNq1010053irJz8PaMwKiF7blCVOw/MvnY6q+qCywU7dLtWNViim0rvnbnrqZJRlodQSGfMsuMTx8IXZ1Y3KuRAYzSgKSxYQvkreYlG6ygCeEbrTZoDPcxfzzaJAdmw7yK6tCYB0SPMRAvjCO+G5KhQNJ7DAzl27caZj4pzzF//Vnbz+c5hzfEMpxUXntpt7x7V++iyMA9ZYq1I/kpzhyz9NDI4jNFPlOX94s9eACMZdFI0iVHcWL5CtjByci5alAI98sYbXpHSfRzAeHaD32Pg1qkiCFEcpAHUFsf67tFeHLn1cUF4RMqHmU8OvSbJkGOONEll2BUgqk7LINQlnA2Qb8HA/AQSb6OBOoEmWSg";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizerImplSubclass vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        fL.setDistancePerPulse(18);
        fR.setDistancePerPulse(18);
        bR.setDistancePerPulse(18);
        bL.setDistancePerPulse(18);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            moveLinear(0.8, 10);
            while (opModeIsActive()) {
                //vuforia.rgb represents the image/frame given by the camera
                if (vuforia.rgb != null) {
                    //converts image to bitmap so that OpenCV can use it to threshold
                    Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

                    //converts bitmap frame to OpenCV Mat class
                    Mat img = new Mat(vuforia.rgb.getHeight(), vuforia.rgb.getWidth(), CvType.CV_8UC3);
                    Utils.bitmapToMat(bm, img);

                    //Pipelines the input frame (has not tested this yet cuz shrika nabbed the control hub
                    //MonishPython pipeline = new MonishPython(telemetry);
                    //Mat threshold = pipeline.processFrame(img);

                    //https://colorpicker.me/ uses H: 0-360, S: 0-100, V: 0-100
                    //but OpenCV uses H: 0-180, S:0-255, V: 0-255
                    //values I got (180, 100, 100) and (250, 100, 100)
                    //but I divided the H by 2 and defined a range:
                    //Scalar blue_lower = new Scalar(80, 50, 25);

                    //detects blue
                    //Scalar blue_lower = new Scalar(90.8333D, 62, 81);
                    //Scalar blue_upper = new Scalar(128.0833D, 255, 255);

                    //trying to detect purple:
                    Mat ROI = img.submat(new Rect(new Point(175, 270), new Point(425, 480)));
                    String roiPath = "sdcard/FIRST/roiPath.png";
                    Imgcodecs.imwrite(roiPath, ROI);

                    Scalar purple_lower = new Scalar(100, 50, 75);
                    Scalar purple_upper = new Scalar(255, 255, 200);

                    //trying to get yellow balls
                    Scalar yellow_lower = new Scalar(0, 62, 128);
                    Scalar yellow_upper = new Scalar(45, 255, 200);
                    //trying to get green
                    Scalar green_lower = new Scalar(45, 0, 75);
                    Scalar green_upper = new Scalar(90, 255, 128);
                    //input image converts to HSV
                    //Mat hsvPic = new Mat();
                    //Imgproc.cvtColor(img, hsvPic, Imgproc.COLOR_RGB2HSV);

                    Mat ROIHsv = new Mat();
                    Imgproc.cvtColor(ROI, ROIHsv, Imgproc.COLOR_RGB2HSV);

                    //the thresholded frame that is black and white and shows the blue in the picture.
                    //Mat threshold = new Mat();
                    //Core.inRange(hsvPic, purple_lower, purple_upper, threshold);

                    Mat threshold = new Mat();
                    Core.inRange(ROIHsv, purple_lower, purple_upper, threshold);
                    //Core.inRange(ROIHsv, purple_lower, purple_upper, ROIThresh);
                    //Core.inRange(ROIHsv, purple_lower, purple_upper, ROIThresh);

                    String roiThreshPath = "sdcard/FIRST/roiThreshPath.png";
                    Imgcodecs.imwrite(roiThreshPath, threshold);

                    Mat threshold2 = new Mat();
                    //Core.inRange(hsvPic, yellow_lower, yellow_upper, threshold2);
                    Core.inRange(ROIHsv, yellow_lower, yellow_upper, threshold2);

                    Mat threshold3 = new Mat();
                    //Core.inRange(hsvPic, green_lower, green_upper, threshold3);
                    Core.inRange(ROIHsv, green_lower, green_upper, threshold3);

                    long whitepixels3 =Core.countNonZero(threshold3);
                    long total = ROIHsv.total();
                    long whitepixels2 =Core.countNonZero(threshold2);
                    long whitepixels1 =Core.countNonZero(threshold);
                    if (whitepixels1>whitepixels2 && whitepixels1>whitepixels3 && whitepixels1*100/total >= 20) {
                        location = SignalSleeveLocation.PURPLE;
                        percent = whitepixels1*100/total;
                    } else if (whitepixels2>whitepixels1 && whitepixels2>whitepixels3 && whitepixels2*100/total >= 20) {
                        location=SignalSleeveLocation.YELLOW;
                        percent = whitepixels2*100/total;
                    } else if (whitepixels3>whitepixels1 && whitepixels3>whitepixels2 && whitepixels3*100/total >= 20) {
                        location=SignalSleeveLocation.GREEN;
                        percent = whitepixels3*100/total;
                    } else {
                        location = SignalSleeveLocation.NONE;
                        percent = 0;
                    }

                    //String rgbPath = "sdcard/FIRST/rgbFile.png";
                    //Imgcodecs.imwrite(rgbPath, hsvPic);

                    //thresholded images should be saved here
                    //String filePath = "sdcard/FIRST/thresholdFile.png";
                    //Imgcodecs.imwrite(filePath, threshold);

                    //String filePath2 = "sdcard/FIRST/threshold2File.png";
                    //Imgcodecs.imwrite(filePath2, threshold2);
                    //String filePath3 = "sdcard/FIRST/threshold3File.png";
                    //Imgcodecs.imwrite(filePath3, threshold3);
                    //use this for extra help: http://overchargedrobotics.org/wp-content/uploads/2018/08/Advanced-Programming-Vision.pdf

                }
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        if (location==SignalSleeveLocation.GREEN){
                            telemetry.addData("location is", "green");
                        }
                        if (location==SignalSleeveLocation.PURPLE){
                            telemetry.addData("location is", "purple");
                        }
                        if (location==SignalSleeveLocation.YELLOW){
                            telemetry.addData("location is", "yellow");
                        }
                        if (location==SignalSleeveLocation.NONE){
                            telemetry.addData("location is", "none");
                        }
                        telemetry.addData("Percentage of color", percent);
                        telemetry.update();
                    }
                }

                //move bobot
                if(!moved) {
                    if (run < 10 || location != SignalSleeveLocation.NONE) {
                        if (location == SignalSleeveLocation.GREEN) {
                            moveLinear(0.8, 9);
                            moveLinear(-0.8, stoppingDistance);
                            moved = true;
                        }
                        if (location == SignalSleeveLocation.PURPLE) {
                            moveLinear(0.8, 9);
                            moveLinear(-0.8, stoppingDistance);
                            turn(-0.8, 60);
                            moveLinear(0.8, 16);
                            moveLinear(-0.8, stoppingDistance);
                            moved = true;
                        }
                        if (location == SignalSleeveLocation.YELLOW) {
                            moveLinear(0.8, 9);
                            moveLinear(-0.8, stoppingDistance);
                            turn(0.8, 60);
                            moveLinear(0.8, 16);
                            moveLinear(-0.8, stoppingDistance);
                            moved = true;
                        }
                        if (location == SignalSleeveLocation.NONE) {
                        }
                    }
                }
                run++;
            }
        }
    }

    public void moveLinear(double power, double distance) {
        // This code will move backward if the power is negative
        // Whenever you call this code add another moveLinear thing for the opposite power and
        // stopping distance
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(-power);
            fR.set(power);
            bL.set(-power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }
    public void strafeLinear(double power, double distance) {
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        while (elapsedTime.milliseconds() < time * 1000) {
            mecanumDrive.driveRobotCentric(-power, 0, 0);
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);
    }


    public void moveLinearTime(double power, double time) {
        // This code will move backward if the power is negative
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(-power);
            fR.set(power);
            bL.set(-power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }
    public void MoveUpAndRight(double power, double distance){
        // Negative power for backright movement
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(power);
            fR.set(0);
            bL.set(0);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }
    public void MoveUpAndLeft(double power, double distance){
        // Negative power for backleft movement
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(0);
            fR.set(power);
            bL.set(power);
            bR.set(0);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }
    public void turnForSec(double power, double time){
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(power);
            fR.set(power);
            bL.set(power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }

    public void turn(double power, double angle){
        // turns counterclockwise negative power for clockwise
        elapsedTime.reset();
        double time  = angle / degPerSec;
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(power);
            fR.set(power);
            bL.set(power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia = new VuforiaLocalizerImplSubclass(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
