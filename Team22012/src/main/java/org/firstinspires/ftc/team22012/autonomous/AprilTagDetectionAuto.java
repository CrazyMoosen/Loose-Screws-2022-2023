package org.firstinspires.ftc.team22012.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team22012.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team22012.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team22012.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp
public class AprilTagDetectionAuto extends LinearOpMode
{
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    double tagsize = 0.166;

    int LEFT = 2;
    int MIDDLE = 6;
    int RIGHT = 16;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-72+7, 27, Math.toRadians(0)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest != null){
            TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-72+7, 27, Math.toRadians(0)))
                    .splineTo(new Vector2d(-12.52+7, 31.86), Math.toRadians(0))
                    .splineTo(new Vector2d(-12.96+7, 23.80), Math.toRadians(270))
                    .splineTo(new Vector2d(-3.15+7, 23.80), Math.toRadians(0.00))
                    .build();

            drive.followTrajectorySequence(untitled0);
//            switch(tagOfInterest.id){
//                case 2: // go straight
//                    TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(-71.25, 36.25, Math.toRadians(0)))
//                            .splineTo(new Vector2d(-36.25, 36.25), Math.toRadians(0.00))
//                            .build();
//
//                    drive.followTrajectorySequence(trajectory);
//                    break;
//                case 6: // go to left
//                    TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(new Pose2d(-71.25, 36.25, Math.toRadians(0)))
//                            .splineTo(new Vector2d(-36.25, 36.25), Math.toRadians(0.00))
//                            .splineTo(new Vector2d(-36.69, 62.41), Math.toRadians(90))
//                            .build();
//
//                    drive.followTrajectorySequence(leftTraj);
//                    break;
//                case 16: // go to right
//                    TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(new Pose2d(-71.25, 36.25, Math.toRadians(0)))
//                            .splineTo(new Vector2d(-36.25, 36.25), Math.toRadians(0.00))
//                            .splineTo(new Vector2d(-37.13, 12.42), Math.toRadians(270))
//                            .build();
//
//                    drive.followTrajectorySequence(rightTraj);
//                    break;
//            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}