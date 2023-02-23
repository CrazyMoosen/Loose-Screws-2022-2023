package org.firstinspires.ftc.team22012.teleoperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;

@TeleOp(name = "TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        DcMotor armMotor2 = hardwareMap.get(DcMotor.class, "linearSlideMotor2");

        Servo armServo1 = hardwareMap.get(Servo.class, "armServo1");
        Servo armServo2 = hardwareMap.get(Servo.class, "armServo2");
        Servo armServo3 = hardwareMap.get(Servo.class, "armServo3");

        armMotor.resetDeviceConfigurationForOpMode();
        armMotor2.resetDeviceConfigurationForOpMode();
        armServo1.resetDeviceConfigurationForOpMode();
        armServo2.resetDeviceConfigurationForOpMode();
        armServo3.resetDeviceConfigurationForOpMode();
        ArmSubsystem arm = new ArmSubsystem(armMotor, armMotor2, armServo1, armServo2, armServo3);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                arm.moveServo1(Math.abs(gamepad1.left_stick_x));
                arm.moveServo2(Math.abs(gamepad1.right_stick_x));
            }
        }
    }
}