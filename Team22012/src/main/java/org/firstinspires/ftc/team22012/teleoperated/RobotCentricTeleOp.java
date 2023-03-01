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

    private DcMotor fL, fR, bL,bR, armMotor, armMotor2;
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
    Servo armServo1, armServo2, armServo3, clawServo;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class,"leftFront");
        fR = hardwareMap.get(DcMotor.class,"rightFront");
        bL = hardwareMap.get(DcMotor.class,"leftRear");
        bR = hardwareMap.get(DcMotor.class,"rightRear");

        bR.resetDeviceConfigurationForOpMode();
        fR.resetDeviceConfigurationForOpMode();
        fL.resetDeviceConfigurationForOpMode();
        bL.resetDeviceConfigurationForOpMode();

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        shreyController = gamepad1;
        monishController = gamepad2;

        armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "linearSlideMotor2");

        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        armServo3 = hardwareMap.get(Servo.class, "armServo3");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        claw = new ClawSubsystem(clawServo);
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor2.resetDeviceConfigurationForOpMode();
        armServo1.resetDeviceConfigurationForOpMode();
        armServo2.resetDeviceConfigurationForOpMode();
        armServo3.resetDeviceConfigurationForOpMode();
        arm = new ArmSubsystem(armMotor, armMotor2, armServo1, armServo2, armServo3);
        claw.closeFully();
        arm.moveServo3(0);
        arm.moveServo1(0);
        arm.moveServo2(0);
        arm.runToPos(0.1);
    }

    @Override
    public void loop() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double speedMultiplier = 1.2;
        //if shrey presses the B button he can boost the speed of the drivetrain
        if (shreyController.b) {
            speedMultiplier = 2;
        }
        mecanumDrive.driveRobotCentric(
                -shreyController.left_stick_x*0.45*speedMultiplier,
                -shreyController.left_stick_y*0.45*speedMultiplier,
                -shreyController.right_stick_x*0.45*speedMultiplier
        );
        if (monishController.right_bumper) {
            claw.closeFully();
        }
        if (monishController.left_bumper) {
            claw.openFully();
        }
        if (monishController.right_trigger >= 0.1 && arm.getHeight() < 30) {
            arm.moveUp();
        }
        else if (monishController.left_trigger >= 0.1) {
            arm.moveDown();
        }
        else {
            if (arm.getHeight() < 30) {
                armMotor.setPower(0.01);
                armMotor2.setPower(0.01);
            }
        }
        if (monishController.y){
            arm.moveServo2(1);
        }
        if (monishController.x){
            arm.moveServo2(0);
        }
        if (monishController.a){
            arm.moveServo3(1);
        }
        if (monishController.b){
            arm.moveServo3(0);
        }
//        if (monishController.dpad_up) {
//            arm.moveServo1(1);
//        }
//        if (monishController.dpad_down) {
//            arm.moveServo1(0);
//        }
        if (monishController.left_stick_x > 0.3) {
            if (arm.getServo1Pos() < 0.99) {
                arm.moveServo1(arm.getServo1Pos() + 0.01);
            }
        }
        if (monishController.left_stick_x < -0.3) {
            if (arm.getServo1Pos() > 0.01) {
                arm.moveServo1(arm.getServo1Pos() - 0.01);
            }
        }

        // Stabillization code

        telemetry.addData("Arm Encoder Distance", armMotor.getCurrentPosition());
        telemetry.addData("Arm Encoder2 Distance", armMotor2.getCurrentPosition());
        telemetry.addData("Arm Encoder Height", arm.getHeight());
        telemetry.addData("bR Position", bR.getCurrentPosition());
        telemetry.addData("Claw Servo Position", claw.getPosition());
        telemetry.addData("Arm Servo1 Position", armServo1.getPosition());
        telemetry.addData("Arm Servo2 Position", armServo2.getPosition());
        telemetry.addData("Arm Servo3 Position", armServo3.getPosition());

        telemetry.update();
    }
}