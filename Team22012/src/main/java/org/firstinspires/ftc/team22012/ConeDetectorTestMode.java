package org.firstinspires.ftc.team22012.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team22012.vision.ConeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This class is used to identify a cone using the camera on the device
 * using {@link ConeDetector} class
 */
@Autonomous(name = "Cone Detector", group = "Auto")
public class ConeDetectorTestMode extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ConeDetector coneDetector = new ConeDetector(telemetry);

        webcam.setPipeline(coneDetector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("CAMERA COULD NOT BE OPENED :(((");
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            //do whatever with the camera lol
        }

        if (gamepad2.a) {
            webcam.stopStreaming();
        }
    }
}
