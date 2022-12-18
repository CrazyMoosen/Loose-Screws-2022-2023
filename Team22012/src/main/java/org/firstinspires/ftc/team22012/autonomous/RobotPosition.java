package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotPosition extends Position {
    Motor fL, fR, bL, bR;
    MecanumDrive mecanumDrive;
//    Motor.Encoder fLEncoder, fREncoder, bLEncoder, bREncoder;
    Direction direction;

    final double speed = 55; // In inches/sec
    final double stoppingDistance = 2.1; // In inches, the distance it takes to stop the robot travelling
    // at the power of 0.6
    final double degPerSec = 150;

    public RobotPosition(Motor fL, Motor fR, Motor bL, Motor bR, double x, double y, Direction direction) {
        super(x, y);
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

//        fLEncoder = fL.encoder;
//        bLEncoder = bL.encoder;
//        fREncoder = fR.encoder;
//        bREncoder = bR.encoder;
        this.direction = direction;

    }

    public void moveToPosManual(double endX, double endY) {
        moveAlongX(endX);
        moveAlongY(endY);
    }

    //moves the robot along the x-axis
    public void moveAlongX(double endX) {
        if (endX != x) {
            double magnitude = (endX - x) / Math.abs(endX - x);
            if (direction == Direction.Right) {
                moveLinear(magnitude * 0.6, Math.abs(endX - x));
                moveLinear(-magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Left) {
                moveLinear(-magnitude * 0.6, Math.abs(endX - x));
                moveLinear(magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Up) {
                strafeLinear(magnitude * 0.6, Math.abs(endX - x));
                strafeLinear(-magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Down) {
                strafeLinear(-magnitude * 0.6, Math.abs(endX - x));
                strafeLinear(magnitude * 0.6, stoppingDistance);
            }
            setX(endX);
        }
    }

    //moves the robot along the y-axis
    public void moveAlongY(double endY) {
        if (endY != y) {
            double magnitude = (endY - y) / Math.abs(endY - y);
            double dist = Math.abs(endY - y);
            if (direction == Direction.Right) {
                strafeLinear(magnitude * 0.6, dist);
                strafeLinear(-magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Left) {
                strafeLinear(-magnitude * 0.6, dist);
                strafeLinear(magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Up) {
                moveLinear(-magnitude * 0.6, dist);
                moveLinear(magnitude * 0.6, stoppingDistance);
            }
            if (direction == Direction.Down) {
                moveLinear(magnitude * 0.6, dist);
                moveLinear(-magnitude * 0.6, stoppingDistance);
            }
            setY(endY);
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
//            fL.set(-power);
//            fR.set(power);
//            bL.set(-power);
//            bR.set(power);
        mecanumDrive.driveRobotCentric(0, -0.6, 0);
        }
       mecanumDrive.driveRobotCentric(0, 0, 0);
//        fL.set(0);
//        fR.set(0);
//        bR.set(0);
//        bL.set(0);
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


    //    public void moveToPos(double endX, double endY) {
//        //moves robot to the x position given
//        //if the x position to goto is to the right of the current x position
//        if (endX > x) {
//            if (direction == Direction.Left) {
//                //move backward until hit x position
//                double distance = fLEncoder.getDistance();
//                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(0.6);
//                    fR.set(-0.6);
//                    bL.set(0.6);
//                    bR.set(-0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Right) {
//                //move forward until hit x position
//                double distance = fLEncoder.getDistance();
//                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(-0.6);
//                    fR.set(0.6);
//                    bL.set(-0.6);
//                    bR.set(0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Up) {
//                //strafe right until hit x position
//                double distance = fLEncoder.getDistance();
//                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Down) {
//                //strafe left until hit x position
//                double distance = fLEncoder.getDistance();
//                while ((endX - x) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//        }
//        //if the x position to goto is to the left of the current x position
//        if (x > endX) {
//            if (direction == Direction.Left) {
//                //move forward until hit x pos
//                double distance = fLEncoder.getDistance();
//                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(-0.6);
//                    fR.set(0.6);
//                    bL.set(-0.6);
//                    bR.set(0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Right) {
//                //move backward until hit x pos
//                double distance = fLEncoder.getDistance();
//                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(0.6);
//                    fR.set(-0.6);
//                    bL.set(0.6);
//                    bR.set(-0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Up) {
//                //strafe left until hit x pos
//                double distance = fLEncoder.getDistance();
//                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Down) {
//                //strafe right until hit x pos
//                double distance = fLEncoder.getDistance();
//                while ((x - endX) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//        }
//
//        //moves robot to the y position given
//        //if the y position to goto is below the current y position
//        if (endY > y) {
//            if (direction == Direction.Right) {
//                //strafe right
//                double distance = fLEncoder.getDistance();
//                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Left) {
//                //strafe left
//                double distance = fLEncoder.getDistance();
//                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Down) {
//                //move forward
//                double distance = fLEncoder.getDistance();
//                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(-0.6);
//                    fR.set(0.6);
//                    bL.set(-0.6);
//                    bR.set(0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Up) {
//                //move backward
//                double distance = fLEncoder.getDistance();
//                while ((endY - y) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(0.6);
//                    fR.set(-0.6);
//                    bL.set(0.6);
//                    bR.set(-0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//        }
//        //if the y position to goto is above the current y position
//        if (y > endY) {
//            if (direction == Direction.Left) {
//                //strafe right
//                double distance = fLEncoder.getDistance();
//                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Right) {
//                //strafe left
//                double distance = fLEncoder.getDistance();
//                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
//                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
//                }
//                mecanumDrive.driveRobotCentric(0, 0, 0);
//            }
//            if (direction == Direction.Up) {
//                //move forward
//                double distance = fLEncoder.getDistance();
//                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(-0.6);
//                    fR.set(0.6);
//                    bL.set(-0.6);
//                    bR.set(0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//            if (direction == Direction.Down) {
//                //move backward
//                double distance = fLEncoder.getDistance();
//                while ((y - endY) > (fLEncoder.getDistance() - distance)) {
//                    fL.set(0.6);
//                    fR.set(-0.6);
//                    bL.set(0.6);
//                    bR.set(-0.6);
//                }
//                fL.set(0);
//                fR.set(0);
//                bL.set(0);
//                bR.set(0);
//            }
//        }
//
//        x = endX;
//        y = endY;
//    }
}
