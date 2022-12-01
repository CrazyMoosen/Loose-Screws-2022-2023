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
import org.opencv.imgproc.Imgproc;

import java.util.List;

//@Autonomous(name = "Vuforia Webcam Detection Blue Team")
@Autonomous(name = "Autonomous Mode")
public class AutonomousMode extends LinearOpMode{

    //The enumeration that is going to be used to detect where to park the robot
    public enum SignalSleeveColor {
        GREEN,
        YELLOW,
        PURPLE,
        NONE;
    }

    private final Rect ROIRect = new Rect(new Point(75, 0), new Point(480, 410));

    //the variable used to hold the color that the camera will detect
    private SignalSleeveColor sleeveColor;

    //this variable will hold the percent of yellow/purple/green detected
    //to further compare against each other and find what color the sleeve
    //actually is
    double percent = 0;
    Motor fL, fR, bL, bR;
    ElapsedTime elapsedTime = new ElapsedTime();

    // Experimentally determined variables
    final double speed = 55; // In inches/sec
    final double stoppingDistance = 2.1; // In inches, the distance it takes to stop the robot travelling
    // at the power of 0.6
    final double degPerSec = 150;

    //testing this just means the robot will start off at Blue 1 position and will face toward the red side.
    private RobotPosition robot = new RobotPosition(hardwareMap, 0, 26, Direction.Right);
    /**
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    //random stuff dont pay attention to this
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
        //Initializes all the motors
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        //sets the distance per pulse so we can move the robot accordingly
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
            //this should do the same thing as below
            robot.moveToPos(robot.getX() + 10, robot.getY());
            //moveLinear(0.6, 10);
            //moveLinear(-0.6, stoppingDistance);
            while (opModeIsActive()) {
                //vuforia.rgb represents the image/frame given by the camera
                if (vuforia.rgb != null) {

                    //converts image to bitmap so that OpenCV can use it to threshold
                    Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

                    //converts bitmap frame to OpenCV Mat class
                    Mat img = new Mat(vuforia.rgb.getHeight(), vuforia.rgb.getWidth(), CvType.CV_8UC3);
                    Utils.bitmapToMat(bm, img);

                    //detects blue
                    //Scalar blue_lower = new Scalar(90.8333D, 62, 81);
                    //Scalar blue_upper = new Scalar(128.0833D, 255, 255);

                    //Gets the Region Of Interest that includes the Signal Cone based on where the robot starts off during the autonomous period
                    Mat ROI = img.submat(ROIRect);

                    //range of values that detects the color purple
                    Scalar purple_lower = new Scalar(100, 50, 75);
                    Scalar purple_upper = new Scalar(255, 255, 200);

                    //range of values that detects the color yellow
                    Scalar yellow_lower = new Scalar(0, 62, 128);
                    Scalar yellow_upper = new Scalar(45, 255, 200);

                    //range of values that detects the color green
                    Scalar green_lower = new Scalar(45, 0, 75);
                    Scalar green_upper = new Scalar(90, 255, 128);

                    Mat ROIHsv = new Mat();
                    Imgproc.cvtColor(ROI, ROIHsv, Imgproc.COLOR_RGB2HSV);

                    Mat thresholdedForPurple = new Mat();
                    Core.inRange(ROIHsv, purple_lower, purple_upper, thresholdedForPurple);

                    Mat thresholdedForYellow = new Mat();
                    Core.inRange(ROIHsv, yellow_lower, yellow_upper, thresholdedForYellow);

                    Mat thresholdedForGreen = new Mat();
                    Core.inRange(ROIHsv, green_lower, green_upper, thresholdedForGreen);

                    long whitePixelsGreen = Core.countNonZero(thresholdedForGreen);
                    long whitePixelsYellow = Core.countNonZero(thresholdedForYellow);
                    long whitePixelsPurple = Core.countNonZero(thresholdedForPurple);

                    double greenValuePercent = Core.sumElems(thresholdedForGreen).val[0] / ROIRect.area() / 255;
                    double yellowValuePercent = Core.sumElems(thresholdedForYellow).val[0] / ROIRect.area() / 255;
                    double purpleValuePercent = Core.sumElems(thresholdedForPurple).val[0] / ROIRect.area() / 255;

                    if (whitePixelsPurple>whitePixelsYellow && whitePixelsPurple>whitePixelsGreen && Math.round(purpleValuePercent*100) >= 20) {
                        sleeveColor = SignalSleeveColor.PURPLE;
                        percent = Math.round(purpleValuePercent*100);
                    }
                    else if (whitePixelsYellow>whitePixelsPurple && whitePixelsYellow>whitePixelsGreen && Math.round(yellowValuePercent*100) >= 20) {
                        sleeveColor = SignalSleeveColor.YELLOW;
                        percent = Math.round(yellowValuePercent * 100);
                    }
                    else if (whitePixelsGreen>whitePixelsPurple && whitePixelsGreen>whitePixelsYellow && Math.round(greenValuePercent*100) >= 20) {
                        sleeveColor = SignalSleeveColor.GREEN;
                        percent = Math.round(greenValuePercent * 100);
                    }
                    else {
                        sleeveColor = SignalSleeveColor.NONE;
                        percent = 0;
                    }
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
                    }
                }

                if (sleeveColor == SignalSleeveColor.GREEN){
                    telemetry.addData("location is", "green");
                }
                if (sleeveColor == SignalSleeveColor.PURPLE){
                    telemetry.addData("location is", "purple");
                }
                if (sleeveColor == SignalSleeveColor.YELLOW){
                    telemetry.addData("location is", "yellow");
                }
                if (sleeveColor == SignalSleeveColor.NONE){
                    telemetry.addData("location is", "none");
                }
                telemetry.addData("Percentage of color", percent);
                telemetry.update();

                //move robot when detected signal sleeve
                if(!moved) {
                    if (run < 10 || sleeveColor != SignalSleeveColor.NONE) {
                        if (sleeveColor == SignalSleeveColor.GREEN) {
                            moveLinear(0.6, 9);
                            moveLinear(-0.6, stoppingDistance);
                            moved = true;
                        }
                        if (sleeveColor == SignalSleeveColor.PURPLE) {
                            moveLinear(-0.6, 10);
                            moveLinear(0.6, stoppingDistance);
                            strafeLinear(0.6, 23);
                            strafeLinear(-0.6, stoppingDistance);
                            moveLinear(0.6, 30);
                            moveLinear(-0.6, stoppingDistance);
                              moved = true;
                        }
                        if (sleeveColor == SignalSleeveColor.YELLOW) {
                            moveLinear(-0.6, 10);
                            moveLinear(0.6, stoppingDistance);
                            strafeLinear(-0.6, 23);
                            strafeLinear(0.6, stoppingDistance);
                            moveLinear(0.6, 30);
                            moveLinear(-0.6, stoppingDistance);
                            moved = true;
                        }
                        if (sleeveColor == SignalSleeveColor.NONE) {
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
    }
}
