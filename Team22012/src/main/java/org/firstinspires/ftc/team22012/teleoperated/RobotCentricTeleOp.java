package org.firstinspires.ftc.team22012.teleoperated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team22012.universal.drive.MecanumDrive;
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

    private DcMotor fL, fR, bL,bR, armMotor;
    private Gamepad shreyController;
    private Gamepad monishController;

    private MecanumDrive mecanumDrive;
    /**
     * This is for the arm and the claw management, once we build the claw
     * and finalize the {@link ClawSubsystem} and {@link ArmSubsystem} hardware
     * classes.
     */
    private ArmSubsystem arm;
    private ClawSubsystem claw;


    private boolean locked = false;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class,"fL");
        fR = hardwareMap.get(DcMotor.class,"fR");
        bL = hardwareMap.get(DcMotor.class,"bL");
        bR = hardwareMap.get(DcMotor.class,"bR");

        bR.resetDeviceConfigurationForOpMode();
        fR.resetDeviceConfigurationForOpMode();
        fL.resetDeviceConfigurationForOpMode();
        bL.resetDeviceConfigurationForOpMode();

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        shreyController = gamepad1;
        monishController = gamepad2;

        armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        claw = new ClawSubsystem(hardwareMap.get(Servo.class, "servo1"), hardwareMap.get(Servo.class, "servo2"));
        armMotor.resetDeviceConfigurationForOpMode();
        arm = new ArmSubsystem(armMotor);

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        double speedMultiplier = 1.2;
        //if shrey presses the B button he can boost the speed of the drivetrain
        if (shreyController.b) {
            speedMultiplier = 1.5;
        }
        mecanumDrive.driveRobotCentric(
                -shreyController.left_stick_x*0.45*speedMultiplier,
                -shreyController.left_stick_y*0.45*speedMultiplier,
                -shreyController.right_stick_x*0.45*speedMultiplier
        );
        double lockedPosition = 0;
        if (monishController.a) {
            locked = true;
        }
        if (monishController.right_bumper) {
            claw.closeFully();
        }
        if (monishController.left_bumper) {
            claw.openFully();
        }

        if (arm.getHeight() > 32) {
            if (arm.getHeight() < 32 && monishController.right_trigger >= 0.1 && !locked) { // if monish/arav presses X button the arm moves up
                arm.moveUp();
            } else if (arm.getHeight() < 32 && monishController.left_trigger >= 0.1 && !locked) { // else if B button down then arm moves down
                arm.moveDown();
            } else if (!locked) {
                arm.stallarm(); //else don't move the arm at all
            }
        }
        else {
            arm.stallarm();
        }

        if (arm.getHeight() < 34 && armMotor.getCurrentPosition() > 32) {
            if (monishController.left_trigger >= 0.1 && !locked) { // else if B button down then arm moves down
                arm.moveDown();
            }
        }

        if ((monishController.left_trigger>=0.1 || monishController.right_trigger>=0.1) && locked) {
            locked = false;
        }


        if (locked) {
            arm.stallarm();
        }

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.addData("Arm Encoder Distance", armMotor.getCurrentPosition());
        telemetry.addData("Arm Encoder Height", arm.getHeight());
        telemetry.addData("bR Position", bR.getCurrentPosition());

        telemetry.update();
    }
}