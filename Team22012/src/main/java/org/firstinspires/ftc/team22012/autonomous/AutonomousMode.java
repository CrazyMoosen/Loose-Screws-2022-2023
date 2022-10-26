package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;
import  java.lang.Math;

/**
    Will use later on; currently disabled, the @Disabled will be commented out during usage.
 **/
@Autonomous(name = "Autonomous")
//@Disabled
public class AutonomousMode extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private ArmSubsystem arm;
    private ClawSubsystem claw;
    private MecanumDrive mecanumDrive;

    // Position tracking
    double movementForward;
    double movementLeft;
    double bearing;
    
    // Coefficients
    final double distancePerPulse = 18;

    private ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        // Setting up the movement type
        fL.setRunMode(Motor.RunMode.PositionControl);
        fR.setRunMode(Motor.RunMode.PositionControl);
        bL.setRunMode(Motor.RunMode.PositionControl);
        bR.setRunMode(Motor.RunMode.PositionControl);

        // Setting the position coefficients, which is the distance traveled per count per revolution
        // Will have to be determined experimentally
        fL.setDistancePerPulse(distancePerPulse);
        fR.setDistancePerPulse(distancePerPulse);
        bL.setDistancePerPulse(distancePerPulse);
        bR.setDistancePerPulse(distancePerPulse);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        waitForStart();
        // Just some test code
        //moveForward(0.8, 90);
        //move(1.0, 90);
        moveForwardv2(1.0, 900);
        telemetry.addData("Position Forward", movementForward);
        telemetry.addData("Position Left", movementLeft);
        telemetry.addData("Angle", bearing);
        /**
        moveBackward(0.8, 18);
        moveForwardv2(0.8, 18);
        telemetry.addData("Position Forward", movementForward);
        telemetry.addData("Position Left", movementLeft);
        telemetry.addData("Angle", bearing);
        moveBackwardv2(0.8, 18);
        moveLeft(0.8, 18);
        telemetry.addData("Position Forward", movementForward);
        telemetry.addData("Position Left", movementLeft);
        telemetry.addData("Angle", bearing);
        moveRight(0.8, 18);
        turnLeft(0.8, 90);
        telemetry.update();
         **/
    }

    public void move(double power, int distance) {
        fL.setTargetDistance((int) (fL.getDistance() + distance));
        fR.setTargetDistance((int) (fR.getDistance() + distance));
        bL.setTargetDistance((int) (bL.getDistance() + distance));
        bR.setTargetDistance((int) (bR.getDistance() + distance));
        telemetry.addData("Target Distance", (int) (fL.getDistance() + distance));

        while (!fL.atTargetPosition() || !fR.atTargetPosition() || !bL.atTargetPosition() || !bR.atTargetPosition()) {
            mecanumDrive.driveRobotCentric(0, -power, 0);
        }
        telemetry.addData("Distance: ", fL.getDistance());
        telemetry.update();
        mecanumDrive.driveRobotCentric(0, 0, 0);
    }

    public void moveForward(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        while (!fL.atTargetPosition()) {
            fL.set(power);
            fR.set(power);
            bL.set(power);
            bR.set(power);
        }
        fL.stopMotor();
        bL.stopMotor();
        fR.stopMotor();
        bR.stopMotor();

        movementForward += fR.getDistance() * cos(bearing);
        movementLeft += fR.getDistance() * sin(bearing);
    }

    public void moveForwardv2(double power, double distance){
        while (bL.getCurrentPosition() < distance) {
            mecanumDrive.driveWithMotorPowers(-power, -power, -power, -power);
        }
        movementForward += fR.getDistance();

        movementForward += fR.getDistance() * cos(bearing);
        movementLeft += fR.getDistance() * sin(bearing);
    }

    public void moveBackward(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        while (!fL.atTargetPosition()) {
            fL.set(-power);
            fR.set(-power);
            bL.set(-power);
            bR.set(-power);
        }
        fL.stopMotor();
        bL.stopMotor();
        fR.stopMotor();
        bR.stopMotor();

        movementForward -= fR.getDistance() * cos(bearing);
        movementLeft -= fR.getDistance() * sin(bearing);
    }

    public void moveBackwardv2(double power, double distance) {
        while (bL.getCurrentPosition() < distance) {
            mecanumDrive.driveWithMotorPowers(-power, -power, -power, -power);
        }
        movementForward -= fR.getDistance();

        movementForward -= fR.getDistance() * cos(bearing);
        movementLeft -= fR.getDistance() * sin(bearing);
    }

    public void moveLeft(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        while (!fL.atTargetPosition()) {
            fL.set(-power);
            fR.set(power);
            bL.set(power);
            bR.set(-power);
        }
        fL.stopMotor();
        bL.stopMotor();
        fR.stopMotor();
        bR.stopMotor();

        movementForward += fR.getDistance() * sin(bearing);
        movementLeft += fR.getDistance() * cos(bearing);
    }

    public void moveRight(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        while (!fL.atTargetPosition()) {
            fL.set(power);
            fR.set(-power);
            bL.set(-power);
            bR.set(power);
        }
        fL.stopMotor();
        bL.stopMotor();
        fR.stopMotor();
        bR.stopMotor();
        movementForward -= fR.getDistance() * sin(bearing);
        movementLeft -= fR.getDistance() * cos(bearing);
    }

    public void turnLeft(double power, double angle){
        // I measured the robot as a rectangle @ 36cm and 10.5cm
        // R = sqrt(18 ** 2 + 5.25 ** 2) = 18.75
        // distance / R = angle in radians
        // distance / R = pi / 180 deg
        // distance = pi / 180 * deg * R
        // Idk what the units are yet lmaoo
        final double R = 18.75;
        double distance = Math.PI / 180 * angle * R;
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        fL.set(-power);
        fR.set(power);
        bL.set(-power);
        bR.set(power);

        bearing += angle;
    }
}
