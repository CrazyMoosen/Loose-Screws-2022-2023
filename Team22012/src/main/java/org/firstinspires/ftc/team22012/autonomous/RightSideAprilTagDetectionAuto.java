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
        Pose2d currentPos;
        if(tagOfInterest != null) {
            // This is the first part of the code, and it garbs the claw and moves the claw up to 7 in
            arm.runToPos(1); //encoder values got offset by 11
            if (clawServo.getPosition() != 1) {
                while (clawServo.getPosition() != 1) {
                    clawServo.setPosition(1.0);
                }
            }
            time.reset();
            while (time.milliseconds() < 900) {}

            arm.moveServo3(0);
            arm.moveServo1(0);
            arm.moveServo2(0);

            arm.runToPos(-4 + 11);

            //TODO: replace with scoreOnHighJunction call
            // This part of the code goes to the high junction moves to the tope of the cone stack, and drops the cone
            currentPos = gotoHighJunction(drive); //up here robot is at (-22, 20.5)
            claw.closeFully();
            arm.moveServo3(0);
            arm.moveServo1(0);
            arm.moveServo2(0);

            arm.runToPos(25 + 11);

            while (arm.getHeight() < 24.8 +  11) {}
            arm.moveServo1(0);
            claw.closeFully();
            currentPos = forward(drive, 3, currentPos); //up here robot is at (-19, 20.5)
            arm.moveServo1(0);
            claw.openFully();

            currentPos = backward(drive, 3, currentPos); //up here robot is at (-22, 20.5)
            arm.stop();
            // Finished with scoring preloaded

            //TODO: replace with scoreConeFromStack call
            //GETTING CONES FROM CONE STACK
            for (int i = 0; i < 5; i++) {
                currentPos = scoreConeFromHighJunction(currentPos, drive, armServo1, arm, clawServo, claw, 9 - 1.75 * i);
            }


            //park from highJunctionPosition
            park(drive, tagOfInterest, currentPos);
            arm.moveServo3(0);
            arm.moveServo1(0);
            arm.moveServo2(0);
            arm.runToPos(0);// go to default position after parking
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

    public Pose2d scoreConeFromHighJunction(Pose2d highJuncPos, SampleMecanumDrive drive, Servo armServo1, ArmSubsystem arm, Servo clawServo, ClawSubsystem claw, double maxConeStackPos) {
        arm.moveServo1(0.3);
        Pose2d currentPos = strafeRight(drive, 42, highJuncPos);
        drive.turn(180);

        arm.runToPos(maxConeStackPos);
        while (arm.getHeight() > maxConeStackPos) {}
//
        if (clawServo.getPosition() != 1) {
            while (clawServo.getPosition() != 1) {
                clawServo.setPosition(1.0);
            }
        }

        time.reset();
        while (time.milliseconds() < 800) {};

        arm.runToPos(26 + 11);
        while (arm.getHeight() < maxConeStackPos + 1.5) {}
        arm.moveUp();

        drive.turn(180);

        currentPos = strafeLeft(drive, 41, drive.getPoseEstimate());
        while (armServo1.getPosition() != 0) {
            armServo1.setPosition(0);
        }

        time.reset();
        while(time.milliseconds() < 400) {}

        currentPos = forward(drive, 3, currentPos);
        time.reset();
        while(time.milliseconds() < 400) {}
        claw.openFully(); // At this point it should have dropped the cone onto the high junction
        currentPos = backward(drive, 3, currentPos); // At this point it should be 3 inches back and have scored the cone
        return currentPos;
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

    public void turn(SampleMecanumDrive drive, double angle) {
        drive.turn(Math.toRadians(angle));
    }
}