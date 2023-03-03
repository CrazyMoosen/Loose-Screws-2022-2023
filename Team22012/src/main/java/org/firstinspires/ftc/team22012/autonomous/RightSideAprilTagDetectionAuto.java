package org.firstinspires.ftc.team22012.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team22012.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team22012.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;
import org.firstinspires.ftc.team22012.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "RightSideAprilTagAutonomousMode")
public class RightSideAprilTagDetectionAuto extends LinearOpMode {
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
    ElapsedTime time = new ElapsedTime();

    double tagsize = 0.166;

    int LEFT = 2;
    int MIDDLE = 6;
    int RIGHT = 16;

    AprilTagDetection tagOfInterest = null;
    DcMotor fL, fR, bL, bR, armMotor, armMotor2;
    Servo armServo1, armServo2, armServo3, clawServo;
    ClawSubsystem claw;
    ArmSubsystem arm;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode()
    {
        fL = hardwareMap.get(DcMotor.class,"leftFront");
        fR = hardwareMap.get(DcMotor.class,"rightFront");
        bL = hardwareMap.get(DcMotor.class,"leftRear");
        bR = hardwareMap.get(DcMotor.class,"rightRear");

        bR.resetDeviceConfigurationForOpMode();
        fR.resetDeviceConfigurationForOpMode();
        fL.resetDeviceConfigurationForOpMode();
        bL.resetDeviceConfigurationForOpMode();

        armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "linearSlideMotor2");

        Servo armServo1 = hardwareMap.get(Servo.class, "armServo1");
        Servo armServo2 = hardwareMap.get(Servo.class, "armServo2");
        Servo armServo3 = hardwareMap.get(Servo.class, "armServo3");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        ClawSubsystem claw = new ClawSubsystem(clawServo);
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor2.resetDeviceConfigurationForOpMode();
        armServo1.resetDeviceConfigurationForOpMode();
        armServo2.resetDeviceConfigurationForOpMode();
        armServo3.resetDeviceConfigurationForOpMode();
        ArmSubsystem arm = new ArmSubsystem(armMotor, armMotor2, armServo1, armServo2, armServo3);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-72, 34, Math.toRadians(0)));
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
        if(tagOfInterest != null) {
            arm.runToPos(1);
            while (time.milliseconds() < 500){
            }
            claw.closeFully();
            arm.moveServo3(0);
            arm.moveServo1(0);
            arm.moveServo2(0);
            time.reset();
            while (time.milliseconds() < 1500){
            }
            arm.runToPos(7);

            Pose2d highJuncPos = gotoHighJunction(drive);
            arm.resetMotorMode();
            while (arm.getHeight() < 36){ // 34 to 36
                arm.moveUp();
            }
            armMotor.setPower(0.01);
            armMotor2.setPower(0.01);
            Pose2d a = forward(drive, 3, highJuncPos);
            claw.openFully();
            time.reset();
            while (time.milliseconds() < 1500){
            }
            arm.runToPos(1);
            arm.stop();

            //park from highJunctionPosition
            park(drive, tagOfInterest, a);
            arm.moveServo3(0);
            arm.moveServo1(0);
            arm.moveServo2(0);
            while (arm.getHeight() > 1) {
                arm.moveDown();
            }
            arm.stop();// go to default position after parking
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
    }

    //cone stack becomes 1.75 inches taller every cone

    //parking from highJunctionPosition
    public void park(SampleMecanumDrive drive, AprilTagDetection tagOfInterest, Pose2d pose) {
        switch (tagOfInterest.id) {
            case 2: // go straight
                Trajectory straightTraj = drive.trajectoryBuilder(pose)
                        .strafeRight(13.5)
                        .build();
                drive.followTrajectory(straightTraj);
                break;
            case 6: // go to left
                Trajectory leftTraj = drive.trajectoryBuilder(pose)
                        .strafeLeft(13.5)
                        .build();
                drive.followTrajectory(leftTraj);

                break;
            case 16: // go to right
                Trajectory rightTraj = drive.trajectoryBuilder(pose)
                        .strafeRight(40) // changed from 37.5 to 40
                        .build();
                Trajectory backTraj = drive.trajectoryBuilder(rightTraj.end())
                        .back(1)
                        .build();
                drive.followTrajectory(rightTraj);
                drive.followTrajectory(backTraj);
                break;
        }
    }
    public Pose2d gotoHighJunction(SampleMecanumDrive drive) {
        TrajectorySequence moveForwardTraj = drive.trajectorySequenceBuilder(new Pose2d(-72, 34, Math.toRadians(0)))
                .splineTo(new Vector2d(-22, 34), Math.toRadians(0))
                .build();
        drive.followTrajectorySequence(moveForwardTraj);

        Trajectory strafeLeftTraj = drive.trajectoryBuilder(moveForwardTraj.end())
                .strafeLeft(13.5)
                .build();
        drive.followTrajectory(strafeLeftTraj);

        return strafeLeftTraj.end();
    }

    public Pose2d forward(SampleMecanumDrive drive, double inches, Pose2d pos) {
        Trajectory forwardTraj = drive.trajectoryBuilder(pos)
                .forward(inches)
                .build();
        drive.followTrajectory(forwardTraj);
        return forwardTraj.end();
    }

    //added backward function
    public Pose2d backward(SampleMecanumDrive drive, double inches, Pose2d pos) {
        Trajectory backwardTraj = drive.trajectoryBuilder(pos)
                .back(inches)
                .build();
        drive.followTrajectory(backwardTraj);
        return backwardTraj.end();
    }

    public Pose2d strafeRight(SampleMecanumDrive drive, double inches, Pose2d startPos) {
        Trajectory strafeRightTraj = drive.trajectoryBuilder(startPos)
                .strafeRight(inches)
                .build();
        drive.followTrajectory(strafeRightTraj);

        return strafeRightTraj.end();
    }
    public Pose2d strafeLeft(SampleMecanumDrive drive, double inches, Pose2d startPos) {
        Trajectory strafeLeftTraj = drive.trajectoryBuilder(startPos)
                .strafeLeft(inches)
                .build();
        drive.followTrajectory(strafeLeftTraj);

        return strafeLeftTraj.end();
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