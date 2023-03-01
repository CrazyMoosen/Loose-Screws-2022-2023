package org.firstinspires.ftc.team22012.teleoperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

@TeleOp(name = "TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        DcMotor armMotor2 = hardwareMap.get(DcMotor.class, "linearSlideMotor2");
        
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor2.resetDeviceConfigurationForOpMode();

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setTargetPosition(0);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo armServo1 = hardwareMap.get(Servo.class, "armServo1");
        Servo armServo2 = hardwareMap.get(Servo.class, "armServo2");
        Servo armServo3 = hardwareMap.get(Servo.class, "armServo3");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        ClawSubsystem claw = new ClawSubsystem(clawServo);
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor2.resetDeviceConfigurationForOpMode();
        armServo1.resetDeviceConfigurationForOpMode();
        armServo2.resetDeviceConfigurationForOpMode();
        armServo3.resetDeviceConfigurationForOpMode();
        ArmSubsystem arm = new ArmSubsystem(armMotor, armMotor2, armServo1, armServo2, armServo3);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                arm.runToHighJunction();

                telemetry.addData("Arm Motor1 Pos:", armMotor.getCurrentPosition());
                telemetry.addData("Arm Motor2 Pos:", armMotor2.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}