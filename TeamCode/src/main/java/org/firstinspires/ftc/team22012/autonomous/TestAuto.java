package org.firstinspires.ftc.team22012.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//package org.firstinspires.ftc.team22012.autonomous;
//
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.vuforia.Image;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.team22012.vision.VuforiaLocalizerImplSubclass;
//import org.opencv.android.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "OpenCV Testing")
//public class TestAuto extends LinearOpMode {
//    private VuforiaLocalizerImplSubclass vuforia;
//    private TFObjectDetector tfod;
//    private static final String VUFORIA_KEY = "AaLoWT//////AAABmWaNq1010053irJz8PaMwKiF7blCVOw/MvnY6q+qCywU7dLtWNViim0rvnbnrqZJRlodQSGfMsuMTx8IXZ1Y3KuRAYzSgKSxYQvkreYlG6ygCeEbrTZoDPcxfzzaJAdmw7yK6tCYB0SPMRAvjCO+G5KhQNJ7DAzl27caZj4pzzF//Vnbz+c5hzfEMpxUXntpt7x7V++iyMA9ZYq1I/kpzhyz9NDI4jNFPlOX94s9eACMZdFI0iVHcWL5CtjByci5alAI98sYbXpHSfRzAeHaD32Pg1qkiCFEcpAHUFsf67tFeHLn1cUF4RMqHmU8OvSbJkGOONEll2BUgqk7LINQlnA2Qb8HA/AQSb6OBOoEmWSg";
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//    private static final String[] LABELS = {
//            "1 Bolt",
//            "2 Bulb",
//            "3 Panel"
//    };
//
//    //this will convert 5-bit to 8-bit
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initVuforia();
//        initTfod();
//
//        if (tfod != null) {
//            tfod.activate();
//            tfod.setZoom(1.0, 16.0 / 9.0);
//        }
//
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (vuforia.rgb != null) {
//                    Image frame = vuforia.rgb;
//                    Bitmap bm = Bitmap.createBitmap(frame.getWidth(), frame.getHeight(), Bitmap.Config.RGB_565);
//                    bm.copyPixelsFromBuffer(frame.getPixels());
//
//                    //converts bitmap frame to OpenCV Mat class
//                    Mat img = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC4);
//                    Utils.bitmapToMat(bm, img);
//
//                    Mat eightbit = new Mat();
//                    Imgproc.cvtColor(img, eightbit, Imgproc.COLOR_RGB2BGR565);
//                    Imgproc.cvtColor(eightbit, eightbit, Imgproc.COLOR_BGR5652BGR);
//                    Imgproc.cvtColor(eightbit, eightbit, Imgproc.COLOR_BGR2BGRA);
//                    Imgproc.cvtColor(eightbit, eightbit, Imgproc.COLOR_BGRA2RGB);
//
//                    Mat hsv = new Mat();
//                    Imgproc.cvtColor(eightbit, hsv, Imgproc.COLOR_BGR2HSV);
//
//                    Scalar blueLower = new Scalar(100, 150, 0);
//                    Scalar blueUpper = new Scalar(140, 255, 255);
//
//                    Mat brownThreshold = new Mat();
//                    Core.inRange(hsv, blueLower, blueUpper, brownThreshold);
//                    Imgcodecs.imwrite("sdcard/FIRST/eightbit.png", eightbit);
//                    Imgcodecs.imwrite("sdcard/FIRST/hsvScan.png", hsv);
//                    Imgcodecs.imwrite("sdcard/FIRST/blueScan.png", brownThreshold);
//
//                }
//            }
//        }
//    }
//
//
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        vuforia = new VuforiaLocalizerImplSubclass(parameters);
//    }
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 300;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
//        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//    }
//
//}
@Autonomous(name = "TestAuto")
public class TestAuto extends OpMode {

    OpenCvWebcam webcam = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new SignalSleeveDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class SignalSleeveDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            telemetry.addLine("Camera opened!");
            telemetry.update();
            Imgcodecs.imwrite("sdcard/FIRST/input.png", input);

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(0, 50, 50);
            Scalar upper = new Scalar(10, 255, 255);

            Mat thresh = new Mat();
            Core.inRange(hsv, lower, upper, thresh);

            return thresh;
        }
    }
}