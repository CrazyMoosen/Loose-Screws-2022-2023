package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);

        shreyController = new GamepadEx(gamepad1);
        //monishController = new GamepadEx(gamepad2);
        //claw = new ClawSubArmSubsystemsystem(hardwareMap);
        //arm = new (hardwareMap);
    }

    @Override
    public void loop() {
        mecanumDrive.driveRobotCentric(
                -shreyController.getLeftX()*0.65,
                -shreyController.getLeftY()*0.65,
                -shreyController.getRightX()*0.65
        );
    }

//    @Override
//    public void loop() {
//        double strafeSpeed = clipRange(shreyController.getLeftX() * 0.65);
//        double forwardSpeed = clipRange(-shreyController.getLeftY() * 0.65);
//        double turnSpeed = clipRange(-shreyController.getRightX() * 0.65);
//
//        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
//        input = input.rotateBy(0.0);
//
//        double theta = input.angle();
//
//        double[] wheelSpeeds = new double[4];
//        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
//        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);
//
//        normalize(wheelSpeeds, input.magnitude());
//
//        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] += turnSpeed;
//        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] -= turnSpeed;
//        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] += turnSpeed;
//        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] -= turnSpeed;
//
//        normalize(wheelSpeeds);
//
//        mecanumDrive.driveWithMotorPowers(
//                wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value * 3 / 4],
//                wheelSpeeds[RobotDrive.MotorType.kFrontRight.value * 3 / 4],
//                wheelSpeeds[RobotDrive.MotorType.kBackLeft.value],
//                wheelSpeeds[RobotDrive.MotorType.kBackRight.value]
//        );
//    }
//
//    public double clipRange(double value) {
//        return value <= rangeMin ? rangeMin
//                : Math.min(value, rangeMax);
//    }
//
//    void normalize(double[] wheelSpeeds, double magnitude) {
//        double maxMagnitude = Math.abs(wheelSpeeds[0]);
//        for (int i = 1; i < wheelSpeeds.length; i++) {
//            double temp = Math.abs(wheelSpeeds[i]);
//            if (maxMagnitude < temp) {
//                maxMagnitude = temp;
//            }
//        }
//        for (int i = 0; i < wheelSpeeds.length; i++) {
//            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
//        }
//
//    }
//
//    void normalize(double[] wheelSpeeds) {
//        double maxMagnitude = Math.abs(wheelSpeeds[0]);
//        for (int i = 1; i < wheelSpeeds.length; i++) {
//            double temp = Math.abs(wheelSpeeds[i]);
//            if (maxMagnitude < temp) {
//                maxMagnitude = temp;
//            }
//        }
//        if (maxMagnitude > 1) {
//            for (int i = 0; i < wheelSpeeds.length; i++) {
//                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
//            }
//        }

}
