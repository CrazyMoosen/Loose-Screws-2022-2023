package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

@TeleOp(name="RobotCentric", group = "Drive Modes")
public class RobotCentricTeleOp extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive mecanumDrive;
    private GamepadEx shreyController;
    private GamepadEx monishController;
    //private CRServo linearSlideServo1;
    //private ArmSubsystem arm;
    //private ClawSubsystem claw;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

       // linearSlideServo1 = hardwareMap.crservo.get("linearSlideServo1");


        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        shreyController = new GamepadEx(gamepad1);
        monishController = new GamepadEx(gamepad2);
        //claw = new ClawSubArmSubsystemsystem(hardwareMap);
        //arm = new (hardwareMap);\
    }

    @Override
    public void loop() {
        mecanumDrive.driveRobotCentric(
                -shreyController.getLeftX()*0.45,
                -shreyController.getLeftY()*0.45,
                -shreyController.getRightX()*0.45
        );
    }
}
