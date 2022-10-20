package org.firstinspires.ftc.team22012.autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

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
        fL.setDistancePerPulse(0.015);
        fR.setDistancePerPulse(0.015);
        bL.setDistancePerPulse(0.015);
        bR.setDistancePerPulse(0.015);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        waitForStart();
        moveForwardDistance(0.8, 18);
        moveBackwardDistance(0.8, 18);
        moveForwardDistancev2(0.8, 18);
        moveBackwardDistancev2(0.8, 18);
    }

    public void moveForwardDistance(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        fL.set(power);
        fR.set(power);
        bL.set(power);
        bR.set(power);
    }

    public void moveForwardDistancev2(double power, double distance){
        while (bL.getCurrentPosition() < distance) {
            mecanumDrive.driveWithMotorPowers(power, power, power, power);
        }
    }

    public void moveBackwardDistance(double power, double distance){
        // Idk what the units are yet lmaoo
        fL.setTargetDistance(distance);
        bL.setTargetDistance(distance);
        fR.setTargetDistance(distance);
        bR.setTargetDistance(distance);

        fL.set(-power);
        fR.set(-power);
        bL.set(-power);
        bR.set(-power);
    }

    public void moveBackwardDistancev2(double power, double distance){
        while (bL.getCurrentPosition() < distance) {
            mecanumDrive.driveWithMotorPowers(-power, -power, -power, -power);
        }
    }
}
