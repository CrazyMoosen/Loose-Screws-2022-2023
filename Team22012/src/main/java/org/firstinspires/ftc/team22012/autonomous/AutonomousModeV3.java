package org.firstinspires.ftc.team22012.autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto v3")
public class AutonomousModeV3 extends LinearOpMode {

    Motor fL, fR, bL, bR;
    ElapsedTime elapsedTime = new ElapsedTime();
    double speed = 33.4646;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        MecanumDrive mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        waitForStart();
        //after play is pressed

        fL.setDistancePerPulse(18);
        fR.setDistancePerPulse(18);
        bR.setDistancePerPulse(18);
        bL.setDistancePerPulse(18);

        //move forward at 1.0 speed for one sec
        moveForward(1.0, 24);
    }

    public void moveForward(double power, double distance) {
        elapsedTime.reset();
        double time = distance / speed;
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
}
