package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;

import java.util.LinkedList;


public class RobotPosition extends Position {
    Motor fL, fR, bL, bR;
    MecanumDrive mecanumDrive;

    Motor.Encoder backRightEncoder;
    Direction direction;

    Bitmap map;
    Canvas mapCanvas;

    LinkedList<Bitmap> frames = new LinkedList<>();

    final double speed = 55; // In inches/sec
//    final double stoppingDistance = 2.1; // In inches, the distance it takes to stop the robot travelling
//    // at the power of 0.6
//    final double degPerSec = 150;

    int[][] lowJunctionPositions = {{48, 24}, {96, 24}, {24, 48}, {120, 48}, {24, 96}, {120, 96}, {48, 120}, {96, 120}};
    int[][] mediumJunctionPositions = {{48, 48}, {96, 48}, {48, 96}, {96, 96}};
    int[][] highJunctionPositions = {{72, 48}, {48, 72}, {96, 72}, {72, 96}};

    //x and y are the bottom left vertex of the robot square no matter what direction it's facing
    public RobotPosition(Motor fL, Motor fR, Motor bL, Motor bR, double x, double y, Direction direction) {
        super(x, y);
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;
        backRightEncoder = bR.encoder;
        backRightEncoder.reset();
        backRightEncoder.setDistancePerPulse(0.0223214286D);

        initPlayingField();
        updatePlayerLocation(x, y);
        saveMap();
        frames.add(map);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        this.direction = direction;

    }
    private void initPlayingField() {
        map = Bitmap.createBitmap(144, 144, Bitmap.Config.ARGB_8888);
        mapCanvas = new Canvas(map);
        Paint color = new Paint();

        //for some reason its ABGR instead of ARGB
        color.setARGB(255, 255, 255, 255);
        mapCanvas.drawRect(new Rect(0, 0, 144, 144), color);
        for (int i = 24; i <= 144; i += 24) {
            color.setARGB(255, 128, 128, 128);
            mapCanvas.drawLine(0, i, 144, i, color);
        }
        for (int j = 24; j <= 144; j += 24) {
            color.setARGB(255, 128, 128, 128);
            mapCanvas.drawLine(j, 0, j, 144, color);
        }

        color.setARGB(255, 0, 0, 0);
        for (int i = 1; i <= 5; i+=2) {
            for (int j = 1; j <= 5; j+=2) {
                mapCanvas.drawCircle(24*i, 24*j, 2.99f, color);
            }
        }

        color.setARGB(255, 0, 255, 0);
        mapCanvas.drawText("L", 48f-2, 24f+4, color);
        mapCanvas.drawText("L", 96f-2, 24f+4, color);
        mapCanvas.drawText("L", 24f-2, 48f+4, color);
        mapCanvas.drawText("L", 120f-2, 48f+4, color);
        mapCanvas.drawText("L", 24f-2, 96f+4, color);
        mapCanvas.drawText("L", 120f-2, 96f+4, color);
        mapCanvas.drawText("L", 48f-2, 120f+4, color);
        mapCanvas.drawText("L", 96f-2, 120f+4, color);

        color.setARGB(255, 255, 255, 0);
        mapCanvas.drawText("M", 48f-4, 48f+2, color);
        mapCanvas.drawText("M", 96f-4, 48f+2, color);
        mapCanvas.drawText("M", 48f-4, 96f+2, color);
        mapCanvas.drawText("M", 96f-4, 96f+2, color);

        color.setARGB(255, 0, 0, 255);
        mapCanvas.drawText("H", 72-4, 48+4, color);
        mapCanvas.drawText("H", 48-4, 72+4, color);
        mapCanvas.drawText("H", 96-4, 72+4, color);
        mapCanvas.drawText("H", 72-4, 96+4, color);

        color.setARGB(255, 0, 0, 255);
        //the lines are actually 10.5 inches but 2 inches thick so should i do 58.5 or 59.5?
        mapCanvas.drawLine(58.5f, 0, 58.5f, 24, color);
        mapCanvas.drawLine(59.5f, 0, 59.5f, 24, color);
        mapCanvas.drawLine(58.5f, 120, 58.5f, 144, color);
        mapCanvas.drawLine(59.5f, 120, 59.5f, 144, color);

        color.setARGB(255, 255, 0, 0);
        mapCanvas.drawLine(85.5f, 0, 85.5f, 24, color);
        mapCanvas.drawLine(84.5f, 0, 84.5f, 24, color);
        mapCanvas.drawLine(85.5f, 120, 85.5f, 144, color);
        mapCanvas.drawLine(84.5f, 120, 84.5f, 144, color);
    }

    private void saveMap() {
        //converts bitmap frame to OpenCV Mat class
        Mat img = new Mat(144, 144, CvType.CV_8UC3);
        Utils.bitmapToMat(this.map, img);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2RGB); //this fixes ARGB being ABGR above

        String filePath = "sdcard/FIRST/map.png";
        Imgcodecs.imwrite(filePath, img);
    }

    public void saveVideo() {
        VideoWriter out = new VideoWriter("sdcard/FIRST/output.avi", VideoWriter.fourcc('M', 'J', 'P', 'G'), 10, new Size(144, 144));
        for (Bitmap frame : frames) {
            Mat frameMatrix = new Mat(144, 144, CvType.CV_8UC3);
            Utils.bitmapToMat(frame, frameMatrix);
            out.write(frameMatrix);
        }
        out.release();
    }

    //this function animates moving to the end location after moving
    private void updatePlayerLocation(double endX, double endY) {
        Paint color = new Paint();
        color.setARGB(255, 255, 0, 0);
        boolean magnitudeX = endX > x;
        if (magnitudeX) {
            for (int i = 0; i < (endX - x); i++) {
                initPlayingField();
                mapCanvas.drawRect(new Rect((int) x + i, (int) y - 14, (int) x + 14 + i, (int) y), color);
                frames.add(map);
            }
        }
        else {
            for (int i = 0; i > (x - endX); i--) {
                initPlayingField();
                mapCanvas.drawRect(new Rect((int) x + i, (int) y - 14, (int) x + 14 + i, (int) y), color);
                frames.add(map);
            }
        }
        boolean magnitudeY = endY > y;
        if (magnitudeY) {
            for (int i = 0; i < (endY - y); i++) {
                initPlayingField();
                mapCanvas.drawRect(new Rect((int)x, (int) y - 14 + i, (int) x + 14 + i, (int) y + i), color);
                frames.add(map);
            }
        }
        else {
            for (int i = 0; i > (y - endY); i--) {
                initPlayingField();
                mapCanvas.drawRect(new Rect((int) x, (int) y - 14 + i, (int) x + 14, (int) y+i), color);
                frames.add(map);
            }
        }
        initPlayingField();
        Rect robot = new Rect((int)x, (int)y-14, (int)x+14, (int)y);
        robot.offset((int)(endX - x), (int)(endY - y));
        mapCanvas.drawRect(robot, color);
        frames.add(map);
    }

    public boolean canMoveToPosition(double endX, double endY) {
        Rect robot = new Rect((int)x, (int)y-14, (int)x+14, (int)y);
        boolean magnitudeX = endX > x;
        if (magnitudeX) {
            for (int i = 0; i < (endX - x); i++) {
                robot.offset(1, 0);
                if (!legiblePosition(robot)) {
                    return false;
                }
            }
        }
        else {
            for (int i = 0; i > (x - endX); i--) {
                robot.offset(-1, 0);
                if (!legiblePosition(robot)) {
                    return false;
                }
            }
        }
        boolean magnitudeY = endY > y;
        if (magnitudeY) {
            for (int i = 0; i < (endY - y); i++) {
                robot.offset(0, 1);
                if (!legiblePosition(robot)) {
                    return false;
                }
            }
        }
        else {
            for (int i = 0; i > (y - endY); i--) {
                robot.offset(0, -1);
                if (!legiblePosition(robot)) {
                    return false;
                }
            }
        }
        return true;
    }

    public boolean legiblePosition(Rect robot) {
        for (int i = 0; i < 8; i++) {
            if (robot.contains(lowJunctionPositions[i][0], lowJunctionPositions[i][1])) {
                return false;
            }
        }
        for (int i = 0; i < 4; i++) {
            if (robot.contains(mediumJunctionPositions[i][0], mediumJunctionPositions[i][1])) {
                return false;
            }
        }
        for (int i = 0; i < 4; i++) {
            if (robot.contains(highJunctionPositions[i][0], highJunctionPositions[i][1])) {
                return false;
            }
        }
        return true;
    }

    public void moveToPos(double endX, double endY) {
        if (canMoveToPosition(endX, endY)) {
            backRightEncoder.reset();
            updatePlayerLocation(endX, endY);
            moveAlongX(endX);
            moveAlongY(endY);
            saveMap();
            saveVideo();
        }
    }

    //moves the robot along the x-axis
    private void moveAlongX(double endX) {
        if (endX != x && endX <= 144 && endX >= 0) {
            boolean magnitude = endX > x;
            if (direction == Direction.Right) {
                moveLinearUsingEncoders(backRightEncoder, mecanumDrive, magnitude, Math.abs(endX - x));
            }
            if (direction == Direction.Left) {
                moveLinearUsingEncoders(backRightEncoder, mecanumDrive, !magnitude, Math.abs(endX - x));
            }
            if (direction == Direction.Up) {
                strafeLinearUsingEncoders(backRightEncoder, mecanumDrive, magnitude, Math.abs(endX - x));
            }
            if (direction == Direction.Down) {
                strafeLinearUsingEncoders(backRightEncoder, mecanumDrive, !magnitude, Math.abs(endX - x));
            }
            setX(endX);
        }
    }

    //moves the robot along the y-axis
    private void moveAlongY(double endY) {
        if (endY != y) {
            boolean magnitude = endY > y;
            double dist = Math.abs(endY - y);
            if (direction == Direction.Right) {
                strafeLinearUsingEncoders(backRightEncoder, mecanumDrive, magnitude, dist);
            }
            if (direction == Direction.Left) {
                strafeLinearUsingEncoders(backRightEncoder, mecanumDrive, !magnitude, dist);
            }
            if (direction == Direction.Up) {
                moveLinearUsingEncoders(backRightEncoder, mecanumDrive, !magnitude, dist);
            }
            if (direction == Direction.Down) {
                moveLinearUsingEncoders(backRightEncoder, mecanumDrive, magnitude, dist);
            }
            setY(endY);
        }
    }

    //maximum position is around 5.6
    //this works no matter if RobotPosition is even initialized
    //it holds the arm in place during teleop or autonomous
    public static void feather(Motor.Encoder armEncoder, ArmSubsystem arm, double minimumRevolutions) {
        armEncoder.setDistancePerPulse(57.6692368);
        if (armEncoder.getRevolutions() < minimumRevolutions - 1) {
            arm.moveUp(); //makes it go up faster if max position
        }
        if (armEncoder.getRevolutions() < minimumRevolutions) {
            arm.gentlyMoveUp();
        }
        else if (armEncoder.getRevolutions() < (minimumRevolutions+0.2) && armEncoder.getRevolutions() > (minimumRevolutions+0.1)) {
            arm.stop();
        }
    }

    //this is accurate very much
    public static void moveLinearUsingEncoders(@NonNull Motor.Encoder bREncoder, MecanumDrive mecanumDrive, boolean forward, double distance) {
        if (forward) {
            while (bREncoder.getRevolutions() < distance / 12.0D) {
                if (bREncoder.getRevolutions() < (distance / 12.0D -1.58 )) {
                    mecanumDrive.driveRobotCentric(0, -0.4, 0); //moves backward
                } else {
                    mecanumDrive.driveRobotCentric(0, -0.2, 0); //moves backward
                }
            }
            mecanumDrive.driveRobotCentric(0, 0, 0);
        }
        else {
            while (bREncoder.getRevolutions() > -distance/12.0D) {
                if (bREncoder.getRevolutions() > ((-distance / 12.0D) + 1.58)) {
                    mecanumDrive.driveRobotCentric(0, 0.4, 0); //moves backward
                } else {
                    mecanumDrive.driveRobotCentric(0, 0.2, 0); //moves backward
                }
            }
            mecanumDrive.driveRobotCentric(0, 0 ,0);
        }
    }

    public static void strafeLinearUsingEncoders(@NonNull Motor.Encoder bREncoder, MecanumDrive mecanumDrive, boolean right, double distance) {
        bREncoder.setDistancePerPulse(42.2983046);
        if (right) {
            while (bREncoder.getRevolutions() <= distance / 12.0D) {
                if (bREncoder.getRevolutions() <= (distance / 12.0)-0.04) {
                    mecanumDrive.driveRobotCentric(-0.5, 0, 0);
                } else {
                    mecanumDrive.driveRobotCentric(-0.1, 0, 0);

                }
            }
            mecanumDrive.driveRobotCentric(0, 0, 0);
        }
        else {
            while (bREncoder.getRevolutions() >= -distance / 12.0D) {
                if (bREncoder.getRevolutions() >= (-distance / 12.0+0.04)) {
                    mecanumDrive.driveRobotCentric(0.5, 0, 0);
                } else {
                    mecanumDrive.driveRobotCentric(0.1, 0, 0);

                }
            }
            mecanumDrive.driveRobotCentric(0, 0, 0);
        }
    }


    public void moveLinear(double power, double distance) {
        ElapsedTime elapsedTime = new ElapsedTime();
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

    // positive power = strafe right
    public void strafeLinear(double power, double distance) {
        ElapsedTime elapsedTime = new ElapsedTime();
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        while (elapsedTime.milliseconds() < time * 1000) {
            mecanumDrive.driveRobotCentric(-power, 0, 0);
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);
    }

    public void turnLeft(MecanumDrive mecanumDrive, BHI260IMU imu) {
//        imu.resetYaw();
//        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//        while (robotOrientation.getYaw(AngleUnit.DEGREES)  >= 270 ) {
//            mecanumDrive.driveRobotCentric(0, 0, 0.3);
//
//        }
//        mecanumDrive.driveRobotCentric(0,0,0);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.milliseconds() < 1050) {
            mecanumDrive.driveRobotCentric(0, 0, 0.4);
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);
        changeDirection(270);
    }

    public void turnRight(MecanumDrive mecanumDrive, BHI260IMU imu) {
//        imu.resetYaw();
//        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//        while (robotOrientation.getYaw(AngleUnit.DEGREES) > -89) {
//            mecanumDrive.driveRobotCentric(0, 0, -0.3);
//        }
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.milliseconds() < 1050) {
            mecanumDrive.driveRobotCentric(0, 0, -0.4);
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);
        changeDirection(90);
    }

    public void turnAround(MecanumDrive mecanumDrive, BHI260IMU imu) {
        imu.resetYaw();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        while (robotOrientation.getYaw(AngleUnit.DEGREES) < 179) {
            mecanumDrive.driveRobotCentric(0, 0, 0.3);
        }
        changeDirection(180);
    }

    //only positive values and in degrees, angle is clockwise of the original angle
    public Direction changeDirection(double angle) {
        if (angle % 360 == 0 || angle % 360 == 90 || angle % 360 == 180 || angle % 360 == 270) {
            if (angle % 360 == 90) {
                if (direction == Direction.Up) {
                    direction = Direction.Right;
                }
                else if (direction == Direction.Right) {
                    direction =Direction.Down;
                }
                else if (direction == Direction.Down) {
                    direction = Direction.Left;
                }
                else {
                    direction = Direction.Up;
                }
            }
            if (angle % 360 == 180) {
                if (direction == Direction.Up) {
                    direction = Direction.Down;
                }
                else if (direction == Direction.Right) {
                    direction = Direction.Left;
                }
                else if (direction == Direction.Down) {
                    direction = Direction.Up;
                }
                else {
                    direction = Direction.Right;
                }
            }
            if (angle % 360 == 270) {
                if (direction == Direction.Up) {
                    direction = Direction.Left;
                }
                else if (direction == Direction.Right) {
                    direction = Direction.Up;
                }
                else if (direction == Direction.Down) {
                    direction = Direction.Right;
                }
                else {
                    direction = Direction.Down;
                }
            }
            return direction;
        }
        else {
            return null;
        }
    }
}
