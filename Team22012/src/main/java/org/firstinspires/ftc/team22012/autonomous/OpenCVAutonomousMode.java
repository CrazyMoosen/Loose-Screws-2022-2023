package org.firstinspires.ftc.team22012.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;
import org.firstinspires.ftc.team22012.vision.SignalSleeveDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "OpenCV Autonomous Mode")
public class OpenCVAutonomousMode extends LinearOpMode {

    double percent = 0;
    DcMotor fL, fR, bL, bR, armMotor;

    ElapsedTime elapsedTime = new ElapsedTime();

    private RobotPosition robot;

    ArmSubsystem arm;
    ClawSubsystem claw;
    OpenCvWebcam webcam = null;
    SignalSleeveDetectionPipeline pipeline = new SignalSleeveDetectionPipeline();


    @Override
    public void runOpMode() throws InterruptedException {
        initWebcam();

        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.addData("X Pos In Robot Position", robot.getX());
//        telemetry.addData("Y Pos In Robot Position", robot.getY());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            elapsedTime.reset();
            while (opModeIsActive()) {
                if (pipeline.getSleeveColor() == SignalSleeveDetectionPipeline.SignalSleeveColor.GREEN) {
                    telemetry.addLine("Green detected!");
                    telemetry.update();
                }
            }
        }
    }

    private void initWebcam() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(pipeline);

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
}
