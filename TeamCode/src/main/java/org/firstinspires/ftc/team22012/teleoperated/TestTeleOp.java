package org.firstinspires.ftc.team22012.teleoperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;

@TeleOp(name = "Test Tele Op")
public class TestTeleOp extends LinearOpMode {

    DcMotor armMotor;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor1");
        armMotor.resetDeviceConfigurationForOpMode();
        ArmSubsystem arm = new ArmSubsystem(armMotor);

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if (opModeIsActive()) {
            arm.runToLowJunction();
            while (opModeIsActive()) {
                if (time.milliseconds() < 6000 && time.milliseconds() > 3000) {
                    arm.runToMidJunction();
                }
                if (time.milliseconds() < 12000 && time.milliseconds() > 6000) {
                    arm.runToHighJunction();
                }
                if (time.milliseconds() > 12000) {
                    arm.runToPos(0);
                }

                telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
                telemetry.addData("Arm Motor Position In Inches", arm.getHeight());
                telemetry.update();
            }
        }
    }
}
