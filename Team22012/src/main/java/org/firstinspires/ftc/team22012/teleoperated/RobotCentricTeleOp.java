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
    //private GamepadEx monishController;
    /**
     * This is for the arm and the claw management, once we build the claw
     * and finalize the {@link ClawSubsystem} and {@link ArmSubsystem} hardware
     * classes.
     */
    private ArmSubsystem arm;
    private ClawSubsystem claw;
    private DriveSubsystem driveOdometry;

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
        //monishController = new GamepadEx(gamepad2);
        claw = new ClawSubsystem(hardwareMap);
        driveOdometry = new DriveSubsystem(hardwareMap);
        //arm = new ArmSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        double speedMultiplier = 1.0;
        if (shreyController.isDown(GamepadKeys.Button.A)) {
            speedMultiplier = 1.5;
        }
        mecanumDrive.driveRobotCentric(
                -shreyController.getLeftX()*0.45*speedMultiplier,
                -shreyController.getLeftY()*0.45*speedMultiplier,
                -shreyController.getRightX()*0.45*speedMultiplier
        );
            claw.close(shreyController.getRightY());
//            if (shreyController.isDown(GamepadKeys.Button.X)){
//                arm.movedown();
//            } else{
//                arm.stop();
//            }
//             if (shreyController.isDown(GamepadKeys.Button.B)){
//            arm.moveup();
//             }else{
//                 arm.stop();
//             }
        telemetry.addData("X Pos According to Odometry", driveOdometry.m_odometry.getPoseMeters().getX() * 39.3701);
        telemetry.addData("Y Pos According to Odometry", driveOdometry.m_odometry.getPoseMeters().getY() * 39.3701);

        telemetry.update();
    }
}
