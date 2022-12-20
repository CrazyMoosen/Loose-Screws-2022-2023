package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;


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

        this.direction = direction;

    }

    public void moveToPosManual(double endX, double endY) {
        moveAlongX(endX);
        moveAlongY(endY);
    }

    //moves the robot along the x-axis
    public void moveAlongX(double endX) {
        if (endX != x && endX <= 144 && endX >= 0) {
            double magnitude = -1;
            if (endX > x) {
                magnitude = 1;
            }
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

    //maximum position is 5.6
    //this works no matter if RobotPosition is even initialized
    //it holds the arm in place during teleop or autonomous
    public static void feather(Motor.Encoder armEncoder, ArmSubsystem arm, double minimumRevolutions) {
        armEncoder.setDistancePerPulse(57.6692368);
        if (armEncoder.getRevolutions() < minimumRevolutions) {
            arm.gentlyMoveUp();
        }
        else if (armEncoder.getRevolutions() < (minimumRevolutions+0.2) && armEncoder.getRevolutions() > (minimumRevolutions+0.1)) {
            arm.stop();
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
