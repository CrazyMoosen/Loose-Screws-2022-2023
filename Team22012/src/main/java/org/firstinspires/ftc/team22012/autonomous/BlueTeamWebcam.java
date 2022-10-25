package org.firstinspires.ftc.team22012.autonomous;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team22012.vision.MonishPython;
import org.firstinspires.ftc.team22012.vision.VuforiaLocalizerImplSubclass;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Vuforia Webcam Detection Blue Team")
public class BlueTeamWebcam extends LinearOpMode{
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
                    Scalar blue_lower = new Scalar(90, 50, 50);
                    Scalar blue_upper = new Scalar(125, 255, 255);

                    //input image converts to HSV
                    Mat hsvPic = new Mat();
                    Imgproc.cvtColor(img, hsvPic, Imgproc.COLOR_RGB2HSV);

                    //the thresholded frame that is black and white and shows the blue in the picture.
                    Mat threshold = new Mat();
                    Core.inRange(hsvPic, blue_lower, blue_upper, threshold);


                    //thresholded image should be saved here
                    String filePath = "sdcard/FIRST/rgbFile.png";
                    Imgcodecs.imwrite(filePath, threshold);

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
                        telemetry.update();
                    }
                }
            }
        }
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
