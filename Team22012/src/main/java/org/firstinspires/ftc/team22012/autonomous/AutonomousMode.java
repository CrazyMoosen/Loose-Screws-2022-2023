package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import android.graphics.Bitmap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;
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

    //Region of Interest that has the signal cone
    private final Rect ROIRect = new Rect(new Point(250, 0), new Point(640, 410));

    //the variable used to hold the color that the camera will detect
    private SignalSleeveColor sleeveColor;

    //this variable will hold the percent of yellow/purple/green detected
    //to further compare against each other and find what color the sleeve
    //actually is
    double percent = 0;
    Motor fL, fR, bL, bR;
    Motor.Encoder bLEncoder, bREncoder, armEncoder;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime gameTimer = new ElapsedTime();

    // Experimentally determined variables
    // i did some mathematics using NO-LOAD RPM so the actual value should be theoretically lower so it matches the case below
    // 312 rotation/min * 1 min/60 seconds * 11.8736494 inches/revolution = 61.74297688 inches/sec
    final double speed = 55; // In inches/sec
    double finalPosition = 0;

    final double stoppingDistance = 2.1; // In inches, the distance it takes to stop the robot travelling
    // at the power of 0.6
    final double degPerSec = 137;

    private RobotPosition robot;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    //random stuff dont pay attention to this
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    int run = 0;
    boolean scored = false;
    boolean parked = false;

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
    private TFObjectDetector tfod;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private MecanumDrive mecanumDrive;
    private BHI260IMU imu;
    @Override
    public void runOpMode() {
        //Initializes all the motors
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        bREncoder = bR.encoder;
        sleeveColor = SignalSleeveColor.NONE;
        claw = new ClawSubsystem(new SimpleServo(hardwareMap, "servo1", 0, 300), new SimpleServo(hardwareMap, "servo2", 0, 300));
        Motor armMotor = new Motor(hardwareMap, "linearSlideMotor1", Motor.GoBILDA.RPM_312);
        arm = new ArmSubsystem(armMotor);
        armEncoder = armMotor.encoder;
        armEncoder.reset();
        //sets the distance per pulse so we can move the robot accordingly
        fL.setDistancePerPulse(0.0223214286D);
        fR.setDistancePerPulse(0.0223214286D); // this is equal to 12/537.6
        bR.setDistancePerPulse(0.0223214286D);
        bL.setDistancePerPulse(0.0223214286D);
        robot = new RobotPosition(fL, fR, bL, bR, 0, 45, Direction.Right);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        mecanumDrive = new MecanumDrive(fL,fR,bL,bR);
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
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, //Orthogonal #9 in the docs
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
        imu.resetYaw();


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("X Pos In Robot Position", robot.getX());
        telemetry.addData("Y Pos In Robot Position", robot.getY());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            elapsedTime.reset();
            gameTimer.reset();
//            while (armEncoder.getRevolutions() < arm.finalPosition) {
//                arm.moveup();
//            }
//            elapsedTime.reset();
//            while (elapsedTime.milliseconds() < 2000) {
//                arm.movedown();
//            }
//            if (armEncoder.getRevolutions() > 0.3) {
//                arm.movedown();
//            }
//            arm.stop();
//            claw.openFully();
//            if (armEncoder.getRevolutions() > 0.10) {
//                arm.movedown();
//            }
            arm.stop();
            claw.closeFully();
            robot.moveToPos(5,45);
            while (opModeIsActive()) {
                //vuforia.rgb represents the image/frame given by the camera
                if (vuforia.rgb != null && sleeveColor != SignalSleeveColor.NONE) {
                    //converts image to bitmap so that OpenCV can use it to threshold
                    Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());

                    //converts bitmap frame to OpenCV Mat class
                    Mat img = new Mat(vuforia.rgb.getHeight(), vuforia.rgb.getWidth(), CvType.CV_8UC3);
                    Utils.bitmapToMat(bm, img);

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

                    String filePath = "sdcard/FIRST/roiPng.png";
                    Imgcodecs.imwrite(filePath, ROI);

                    long whitePixelsGreen = Core.countNonZero(thresholdedForGreen);
                    long whitePixelsYellow = Core.countNonZero(thresholdedForYellow);
                    long whitePixelsPurple = Core.countNonZero(thresholdedForPurple);

                    double greenValuePercent = Core.sumElems(thresholdedForGreen).val[0] / ROIRect.area() / 255;
                    double yellowValuePercent = Core.sumElems(thresholdedForYellow).val[0] / ROIRect.area() / 255;
                    double purpleValuePercent = Core.sumElems(thresholdedForPurple).val[0] / ROIRect.area() / 255;

                    if (whitePixelsPurple > whitePixelsYellow && whitePixelsPurple > whitePixelsGreen && Math.round(purpleValuePercent * 100) >= 20) {
                        sleeveColor = SignalSleeveColor.PURPLE;
                        percent = Math.round(purpleValuePercent * 100);
                    } else if (whitePixelsYellow > whitePixelsPurple && whitePixelsYellow > whitePixelsGreen && Math.round(yellowValuePercent * 100) >= 20) {
                        sleeveColor = SignalSleeveColor.YELLOW;
                        percent = Math.round(yellowValuePercent * 100);
                    } else if (whitePixelsGreen > whitePixelsPurple && whitePixelsGreen > whitePixelsYellow && Math.round(greenValuePercent * 100) >= 20) {
                        sleeveColor = SignalSleeveColor.GREEN;
                        percent = Math.round(greenValuePercent * 100);
                    } else {
                        sleeveColor = SignalSleeveColor.NONE;
                        percent = 0;
                    }
                }
                if (sleeveColor == SignalSleeveColor.GREEN) {
                    telemetry.addData("location is", "green");
                }
                if (sleeveColor == SignalSleeveColor.PURPLE) {
                    telemetry.addData("location is", "purple");
                }
                if (sleeveColor == SignalSleeveColor.YELLOW) {
                    telemetry.addData("location is", "yellow");
                }
                if (sleeveColor == SignalSleeveColor.NONE) {
                    telemetry.addData("location is", "none");
                }
                telemetry.addData("Percentage of color", percent);
                elapsedTime.reset();
                while(elapsedTime.milliseconds() < 300 && opModeIsActive()){

                }
                elapsedTime.reset();
                while(elapsedTime.milliseconds() < 300 && opModeIsActive()){

                }
                if (!scored && opModeIsActive()) {
                    robot.moveToPos(5, 41);
                    scoreOnHighJunction();
                    scored = true;
                }
                elapsedTime.reset();
                while(elapsedTime.milliseconds() < 300 && opModeIsActive()){}
                if (!parked&&opModeIsActive()){
                    park();
                    parked=true;
                }
                telemetry.addData("X Pos In Robot Position", robot.getX());
                telemetry.addData("Y Pos In Robot Position", robot.getY());
                telemetry.update();
            }
        }
    }
    public void scoreOnHighJunction() {
        while (armEncoder.getRevolutions() < arm.finalPosition-0.2) {
            arm.moveUp();
        }
        arm.stallarm();
        robot.moveToPos(51.5, 41);
        robot.moveToPos(51.5, 53.75);
        robot.moveToPos(55, 53.75);
        elapsedTime.reset();
        claw.openFully();
        while (elapsedTime.milliseconds() <= 500){}
        //arm.moveDown();
        //arm.stop();
    }
    public void center(){
        double centerX = robot.getX() - (robot.getX() % 24) + 5;
        double centerY = robot.getY() - (robot.getY() % 24) + 19;
        robot.moveToPos(centerX, centerY);
    }
    public void park(){
        //center();
        if (sleeveColor==SignalSleeveColor.GREEN){
            // Changed from 48 to 50
//            robot.moveToPos(52,42);
        }else if (sleeveColor==SignalSleeveColor.PURPLE){

        }else if (sleeveColor==SignalSleeveColor.YELLOW){
//            robot.moveToPos(52,2);
        }
    }
    public void getCone(){
        robot.moveToPos(51,24);
        //TURN NEEDED

        claw.openFully();
        elapsedTime.reset();
        while(elapsedTime.milliseconds()<=1000) {
            arm.moveUp();
        }
        robot.moveToPos(68, 17);
        elapsedTime.reset();
        while(elapsedTime.milliseconds()<=750) {
            arm.moveDown();
        }
        claw.closeFully();
//TURN NEEDED

        center();
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
        elapsedTime.reset();
    }
    public void strafeLinear(double power, double distance) {
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        while (elapsedTime.milliseconds() < time * 1000) {
            mecanumDrive.driveRobotCentric(-power, 0, 0);
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);

        elapsedTime.reset();
    }

    public void turn(double power, double angle){
        // turns counterclocise negative power for clockwise
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

        robot.changeDirection(360-angle);
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
