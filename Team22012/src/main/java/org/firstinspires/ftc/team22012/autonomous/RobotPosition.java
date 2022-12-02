package org.firstinspires.ftc.team22012.autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotPosition extends Position {

    Motor fL, fR, bL, bR;
    Motor.Encoder fLEncoder, fREncoder, bLEncoder, bREncoder;
    HardwareMap hardwareMap;
    Direction direction;
    public RobotPosition(HardwareMap hardwareMap, double x, double y, Direction direction) {
        super(x, y);
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        fLEncoder = fL.encoder;
        bLEncoder = bL.encoder;
        fREncoder = fR.encoder;
        bREncoder = bR.encoder;
        this.hardwareMap = hardwareMap;
        this.direction = direction;

    }
    public void moveToPos(double endX, double endY) {
        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        //moves robot to the x position given
        //if the x position to goto is to the right of the current x position
        if (endX > x) {
            if (direction == Direction.Left) {
                //move backward until hit x position
                double distance = fLEncoder.getDistance();
                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
                    fL.set(0.6);
                    fR.set(-0.6);
                    bL.set(0.6);
                    bR.set(-0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Right) {
                //move forward until hit x position
                double distance = fLEncoder.getDistance();
                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
                    fL.set(-0.6);
                    fR.set(0.6);
                    bL.set(-0.6);
                    bR.set(0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Up) {
                //strafe right until hit x position
                double distance = fLEncoder.getDistance();
                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Down) {
                //strafe left until hit x position
                double distance = fLEncoder.getDistance();
                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
        }
        //if the x position to goto is to the left of the current x position
        if (x > endX) {
            if (direction == Direction.Left) {
                //move forward until hit x pos
                double distance = fLEncoder.getDistance();
                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
                    fL.set(-0.6);
                    fR.set(0.6);
                    bL.set(-0.6);
                    bR.set(0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Right) {
                //move backward until hit x pos
                double distance = fLEncoder.getDistance();
                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
                    fL.set(0.6);
                    fR.set(-0.6);
                    bL.set(0.6);
                    bR.set(-0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Up) {
                //strafe left until hit x pos
                double distance = fLEncoder.getDistance();
                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Down) {
                //strafe right until hit x pos
                double distance = fLEncoder.getDistance();
                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
        }

        //moves robot to the y position given
        //if the y position to goto is below the current y position
        if (endY > y) {
            if (direction == Direction.Right) {
                //strafe right
                double distance = fLEncoder.getDistance();
                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Left) {
                //strafe left
                double distance = fLEncoder.getDistance();
                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Down) {
                //move forward
                double distance = fLEncoder.getDistance();
                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
                    fL.set(-0.6);
                    fR.set(0.6);
                    bL.set(-0.6);
                    bR.set(0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Up) {
                //move backward
                double distance = fLEncoder.getDistance();
                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
                    fL.set(0.6);
                    fR.set(-0.6);
                    bL.set(0.6);
                    bR.set(-0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
        }
        //if the y position to goto is above the current y position
        if (y > endY) {
            if (direction == Direction.Left) {
                //strafe right
                double distance = fLEncoder.getDistance();
                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Right) {
                //strafe left
                double distance = fLEncoder.getDistance();
                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
                }
                mecanumDrive.driveRobotCentric(0, 0, 0);
            }
            if (direction == Direction.Up) {
                //move forward
                double distance = fLEncoder.getDistance();
                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
                    fL.set(-0.6);
                    fR.set(0.6);
                    bL.set(-0.6);
                    bR.set(0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
            if (direction == Direction.Down) {
                //move backward
                double distance = fLEncoder.getDistance();
                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
                    fL.set(0.6);
                    fR.set(-0.6);
                    bL.set(0.6);
                    bR.set(-0.6);
                }
                fL.set(0);
                fR.set(0);
                bL.set(0);
                bR.set(0);
            }
        }

        x = endX;
        y = endY;
    }
}
