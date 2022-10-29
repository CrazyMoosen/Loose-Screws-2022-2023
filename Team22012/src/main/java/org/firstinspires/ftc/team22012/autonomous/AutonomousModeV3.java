package org.firstinspires.ftc.team22012.autonomous;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto v3")
public class AutonomousModeV3 extends LinearOpMode {

    Motor fL, fR, bL, bR;
    ElapsedTime elapsedTime = new ElapsedTime();

    // Experimentally determined variables
    double speed = 50.5; // In inches/sec
    double stoppingDistance = 1; // In inches, the distance it takes to stop the robot travelling
    // at the power of 0.8
    double degPerSec = 160;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        fL.setDistancePerPulse(18);
        fR.setDistancePerPulse(18);
        bR.setDistancePerPulse(18);
        bL.setDistancePerPulse(18);

        waitForStart();
        //after play is pressed
        //move forward at 1.0 speed for one sec
        moveLinear(0.8, 19);
        moveLinear(-0.8, stoppingDistance);
        turn(0.8, 80);
        moveLinear(0.8, 16);
        moveLinear(-0.8, stoppingDistance);
        telemetry.addData("fL says:", fL.encoder.getDistance());
        telemetry.addData("fR says:", fR.encoder.getDistance());
        telemetry.addData("bL says:", bL.encoder.getDistance());
        telemetry.addData("bR says:", bR.encoder.getDistance());
        telemetry.addData("Num Revs: ", fL.encoder.getRevolutions());
        telemetry.update();
    }

    public void moveLinear(double power, double distance) {
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
    }

    public void moveLinearTime(double power, double time) {
        // This code will move backward if the power is negative
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
    }

    public void moveRight(double power, double distance){
        double time = distance / (speed * abs(power));
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(-power);
            fR.set(power);
            bL.set(power);
            bR.set(-power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }

    public void turnForSec(double power, double time){
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(power);
            fR.set(power);
            bL.set(power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }

    public void turn(double power, double angle){
        elapsedTime.reset();
        double time  = angle / degPerSec;
        while (elapsedTime.milliseconds() < time * 1000) {
            fL.set(power);
            fR.set(power);
            bL.set(power);
            bR.set(power);
        }
        fL.set(0);
        fR.set(0);
        bR.set(0);
        bL.set(0);
    }
}
