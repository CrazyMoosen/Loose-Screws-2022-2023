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

        fL.setDistancePerPulse(18.0); // Set to 18mm per pulse

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        shreyController = new GamepadEx(gamepad1);
        //monishController = new GamepadEx(gamepad2);

        //claw = new ClawSubsystem(hardwareMap);
        //arm = new ArmSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        double distance = fL.getDistance();
        double leftX = shreyController.getLeftX();
        double rightX = shreyController.getRightX();
        double leftY = shreyController.getLeftY();

        if (leftX > 0) {
            leftX = Math.pow(leftX, 2);
        }
        else {
            leftX = -1 * Math.pow(leftX, 2);
        }

        if (rightX > 0) {
            rightX = Math.pow(rightX, 2);
        }
        else {
            rightX = -1 * Math.pow(rightX, 2);
        }

        if (leftY > 0) {
            leftY = Math.pow(leftY, 2);
        }
        else {
            leftY = -1 * Math.pow(leftY, 2);
        }


        //if (shreyController.getLeftX() > 0D)
        mecanumDrive.driveRobotCentric(
                -leftX,
                -leftY,
                -rightX
        );
        //claw.close(monishController.getRightX());
        telemetry.addData("Position:", String.valueOf(distance));
        telemetry.update();
        if(monishController.isDown(GamepadKeys.Button.A)){
            arm.completeRetract();
        }
        if(monishController.isDown(GamepadKeys.Button.Y)){
            arm.completeExtend();
        }
        if(!monishController.isDown(GamepadKeys.Button.Y) && !monishController.isDown(GamepadKeys.Button.A)){
            arm.ArmMove(height);
        }
        if (monishController.isDown(GamepadKeys.Button.X)){
            claw.release();
        }
        if (monishController.isDown(GamepadKeys.Button.B)){
            claw.close(closeValue);
        }
    }
}
