package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;


public class RobotPosition extends Position {
    Motor fL, fR, bL, bR;
    MecanumDrive mecanumDrive;
//    Motor.Encoder fLEncoder, fREncoder, bLEncoder, bREncoder;
    Motor.Encoder backRightEncoder;
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
        backRightEncoder = bR.encoder;
        backRightEncoder.reset();
        backRightEncoder.setDistancePerPulse(0.0223214286D);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        this.direction = direction;

    }

    public void moveToPos(double endX, double endY) {
        backRightEncoder.reset();
        moveAlongX(endX);
        moveAlongY(endY);
    }

    //moves the robot along the x-axis
    public void moveAlongX(double endX) {
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
    public void moveAlongY(double endY) {
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
            arm.moveup(); //makes it go up faster if max position
        }
        if (armEncoder.getRevolutions() < minimumRevolutions) {
            arm.gentlyMoveUp();
        }
        else if (armEncoder.getRevolutions() < (minimumRevolutions+0.2) && armEncoder.getRevolutions() > (minimumRevolutions+0.1)) {
            arm.stop();
        }
    }

    //this is accurate very much
    public static void moveLinearUsingEncoders(Motor.Encoder bREncoder, MecanumDrive mecanumDrive, boolean forward, double distance) {
        if (forward) {
            while (bREncoder.getRevolutions() < distance / 12.0D) {
                if (bREncoder.getRevolutions() < (distance / 12.0D - 1.3)) {
                    mecanumDrive.driveRobotCentric(0, -0.6, 0); //moves backward
                } else {
                    mecanumDrive.driveRobotCentric(0, -0.1, 0); //moves backward
                }
            }
            mecanumDrive.driveRobotCentric(0, 0, 0);
        }
        else {
            while (bREncoder.getRevolutions() > -distance/12.0D) {
                if (bREncoder.getRevolutions() > ((-distance / 12.0D) + 1.3)) {
                    mecanumDrive.driveRobotCentric(0, 0.6, 0); //moves backward
                } else {
                    mecanumDrive.driveRobotCentric(0, 0.1, 0); //moves backward
                }
            }
            mecanumDrive.driveRobotCentric(0, 0 ,0);
        }
    }

    public void strafeLinearUsingEncoders(Motor.Encoder bREncoder, MecanumDrive mecanumDrive, boolean right, double distance) {
        bREncoder.setDistancePerPulse(42.2983046);
        if (right) {
            while (bREncoder.getRevolutions() <= distance / 12.0D) {
                if (bREncoder.getRevolutions() <= (distance / 12.0 - 0.2D)) {
                    mecanumDrive.driveRobotCentric(-0.6, 0, 0);
                } else {
                    mecanumDrive.driveRobotCentric(-0.1, 0, 0);

                }
            }
            mecanumDrive.driveRobotCentric(0, 0, 0);
        }
        else {
            while (bREncoder.getRevolutions() >= -distance / 12.0D) {
                if (bREncoder.getRevolutions() >= (-distance / 12.0 + 0.3D)) {
                    mecanumDrive.driveRobotCentric(0.6, 0, 0);
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
        imu.resetYaw();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        while (robotOrientation.getYaw(AngleUnit.DEGREES) < 89) {
            mecanumDrive.driveRobotCentric(0, 0, 0.3);
        }
        changeDirection(270);
    }

    public void turnRight(MecanumDrive mecanumDrive, BHI260IMU imu) {
        imu.resetYaw();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        while (robotOrientation.getYaw(AngleUnit.DEGREES) > -89) {
            mecanumDrive.driveRobotCentric(0, 0, -0.3);
        }
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
