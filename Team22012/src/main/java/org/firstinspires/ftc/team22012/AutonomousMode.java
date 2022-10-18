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

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        arm = new ArmSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            runTime = new ElapsedTime(0);
            runTime.startTime();
            //Does whatever it does in autonomous mode
            if (runTime.seconds() < 5) {
                mecanumDrive.driveRobotCentric(0, 1.0, 0);
            }
        }
    }
}
