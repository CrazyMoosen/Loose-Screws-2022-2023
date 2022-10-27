package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

@TeleOp(name="RobotCentric", group = "Drive Modes")
public class RobotCentricTeleOp extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive mecanumDrive;
    private GamepadEx shreyController;
    //private GamepadEx monishController;

    /**
     * This is for the arm and the claw management, once we build the claw
     * and finalize the {@link ClawSubsystem} and {@link ArmSubsystem} hardware
     * classes.
     */
    //private ArmSubsystem arm;
    //private ClawSubsystem claw;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        shreyController = new GamepadEx(gamepad1);
        //monishController = new GamepadEx(gamepad2);

        //claw = new ClawSubsystem(hardwareMap);
        //arm = new ArmSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        mecanumDrive.driveRobotCentric(
                shreyController.getLeftX()*0.5,
                -shreyController.getLeftY()*0.5,
                -shreyController.getRightX()*0.5
        );
    }
}
