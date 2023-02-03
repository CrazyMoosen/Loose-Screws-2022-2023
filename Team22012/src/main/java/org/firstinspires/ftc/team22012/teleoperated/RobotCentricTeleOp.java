package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22012.autonomous.RobotPosition;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

@TeleOp(name="RobotCentric", group = "Drive Modes")
public class RobotCentricTeleOp extends OpMode {


    public static final double kDefaultRangeMin = -1.0;
    public static final double kDefaultRangeMax = 1.0;
    public static final double kDefaultMaxSpeed = 1.0;

    protected double rangeMin = kDefaultRangeMin;
    protected double rangeMax = kDefaultRangeMax;
    protected double maxOutput = kDefaultMaxSpeed;

    private Motor fL, fR, bL, bR;
    Motor.Encoder fLEncoder, fREncoder, bLEncoder, bREncoder;
    private MecanumDrive mecanumDrive;
    private GamepadEx shreyController;
    private GamepadEx monishController;
    /**
     * This is for the arm and the claw management, once we build the claw
     * and finalize the {@link ClawSubsystem} and {@link ArmSubsystem} hardware
     * classes.
     */
    private ArmSubsystem arm;
    private Motor.Encoder armEncoder;
    private ClawSubsystem claw;

    private boolean locked = false;
    private double lockedPosition = 0;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        bREncoder = bR.encoder;
        bREncoder.reset();
        shreyController = new GamepadEx(gamepad1);
        monishController = new GamepadEx(gamepad2);

        Motor armMotor = new Motor(hardwareMap, "linearSlideMotor1", Motor.GoBILDA.RPM_312);
        claw = new ClawSubsystem(new SimpleServo(hardwareMap, "servo1", 0, 300), new SimpleServo(hardwareMap, "servo2", 0, 300));
        arm = new ArmSubsystem(armMotor);
        armEncoder = armMotor.encoder;
        armEncoder.reset();

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        bREncoder.setDistancePerPulse(0.0223214286D); //this shows correct distance in inches
        double speedMultiplier = 1.2;
        //if shrey presses the B button he can boost the speed of the drivetrain
        if (shreyController.isDown(GamepadKeys.Button.B)) {
            speedMultiplier = 1.5;
        }
        mecanumDrive.driveRobotCentric(
                -shreyController.getLeftX()*0.45*speedMultiplier,
                -shreyController.getLeftY()*0.45*speedMultiplier,
                -shreyController.getRightX()*0.45*speedMultiplier
        );
        if (monishController.isDown(GamepadKeys.Button.A)) {
            locked = true;
            lockedPosition = armEncoder.getRevolutions();
        }
        if (monishController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            claw.closeFully();
        }
        if (monishController.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            claw.openFully();
        }

        if (armEncoder.getRevolutions() <= 5.6 && monishController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>=0.1 && !locked) { // if monish/arav presses X button the arm moves up
            arm.moveUp();
        }
        else if (armEncoder.getRevolutions() <= 5.6 && monishController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>=0.1 && !locked){ // else if B button down then arm moves down
            arm.moveDown();
        }
        else if (!locked){
            arm.stop(); //else don't move the arm at all
        }

        if ((monishController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>=0.1 || monishController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>=0.1) && locked) {
            locked = false;
            lockedPosition = 0;
        }

        //this is everything that monish/arav controls
//        if (monishController.isDown(GamepadKeys.Button.A)) {
//            locked = true;
//            lockedPosition = armEncoder.getRevolutions();
//        }
//
//        if (monishController.isDown(GamepadKeys.Button.X) && !locked) { // if monish/arav presses X button the arm moves up
//            arm.moveup();
//        }
//        else if (monishController.isDown(GamepadKeys.Button.B) && !locked){ // else if B button down then arm moves down
//            arm.movedown();
//        }
//        else if (!locked){
//            arm.stop(); //else don't move the arm at all
//        }
//
//        if ((monishController.isDown(GamepadKeys.Button.X) || monishController.isDown(GamepadKeys.Button.B)) && locked) {
//            locked = false;
//            lockedPosition = 0;
//        }
//
//        if (monishController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
//            claw.closeFully();
//        }
//        if (monishController.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
//            claw.openFully();
//        }
        if (shreyController.isDown(GamepadKeys.Button.Y)) {
            claw.closeForBeacon();
        }

        if (locked) {
            RobotPosition.feather(armEncoder, arm, lockedPosition);
        }

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.addData("Arm Encoder Distance", armEncoder.getDistance());
        telemetry.addData("Arm Encoder Revolutions", armEncoder.getRevolutions());
        telemetry.addData("bR CPR", bR.getCPR());
        telemetry.addData("bR Position", bREncoder.getPosition());
        telemetry.addData("bR Revolutions", bREncoder.getRevolutions());
        telemetry.addData("bR Distance", bREncoder.getDistance());

        telemetry.update();
    }
}
