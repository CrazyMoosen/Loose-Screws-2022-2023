package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22012.autonomous.RobotPosition;
import org.firstinspires.ftc.team22012.universal.subsystems.ArmSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;
import org.firstinspires.ftc.team22012.universal.subsystems.DriveSubsystem;

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
        fLEncoder = fL.encoder;
        bLEncoder = bL.encoder;
        fREncoder = fR.encoder;
        bREncoder = bR.encoder;
        shreyController = new GamepadEx(gamepad1);
        monishController = new GamepadEx(gamepad2);

        Motor armMotor = new Motor(hardwareMap, "linearSlideMotor1", Motor.GoBILDA.RPM_312);
        claw = new ClawSubsystem(new SimpleServo(hardwareMap, "servo1", 0, 300), new SimpleServo(hardwareMap, "servo2", 0, 300));
        arm = new ArmSubsystem(armMotor);
        this.armEncoder = armMotor.encoder;
        armEncoder.reset();

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        armEncoder.setDistancePerPulse(57.6692368);
        double speedMultiplier = 1.0;
        //if shrey presses the B button he can boost the speed of the drivetrain
        if (shreyController.isDown(GamepadKeys.Button.B)) {
            speedMultiplier = 1.5;
        }
        mecanumDrive.driveRobotCentric(
                -shreyController.getLeftX()*0.45*speedMultiplier,
                -shreyController.getLeftY()*0.45*speedMultiplier,
                -shreyController.getRightX()*0.45*speedMultiplier
        );

        //this is everything that monish controls
        if (monishController.isDown(GamepadKeys.Button.A)) {
            locked = true;
            lockedPosition = armEncoder.getRevolutions();
        }

        if (monishController.isDown(GamepadKeys.Button.X) && !locked) { // if monish/arav presses B button the arm moves up
            arm.moveup();
        }
        else if (monishController.isDown(GamepadKeys.Button.B) && !locked){ // else if A button down then arm moves down
            arm.movedown();
        }
        else if (!locked){
            arm.stop(); //else don't move the arm at all
        }

        if ((monishController.isDown(GamepadKeys.Button.X) || monishController.isDown(GamepadKeys.Button.B)) && locked) {
            locked = false;
            lockedPosition = 0;
        }

        if (monishController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            claw.closeFully();
        }
        if (monishController.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            claw.openFully();
        }
        if (monishController.isDown(GamepadKeys.Button.Y)) {
            claw.closeForBeacon();
        }

        if (locked) {
            RobotPosition.feather(armEncoder, arm, lockedPosition);
        }


//        telemetry.addData("X Pos According to Odometry", driveOdometry.m_odometry.getPoseMeters().getX() * 39.3701);
//        telemetry.addData("Y Pos According to Odometry", driveOdometry.m_odometry.getPoseMeters().getY() * 39.3701);

        telemetry.addData("Left Servo Position", claw.getLeftServoAngle());
        telemetry.addData("Right Servo Position", claw.getRightServoAngle());
        telemetry.addData("Arm Encoder Distance", armEncoder.getDistance());
        telemetry.addData("Arm Encoder Revolutions", armEncoder.getRevolutions());

        telemetry.update();
    }
}
