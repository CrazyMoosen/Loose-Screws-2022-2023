package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentric", group = "Drive Modes")
public class FieldCentricTeleOp extends OpMode {
    //https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html
    //this stuff slaps bro

    private BHI260IMU imu;
    private MecanumDrive mecanumDrive;
    private GamepadEx shreyController;

    @Override
    public void init() {
        Motor fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        Motor bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        Motor bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        //need FTC Version 8.1 to get it. 8.1 DROPPED LETS GOOOO
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, //Orthogonal #9 in the docs
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        //this here set it to radians before, however in mecanumDrive.driveFieldCentric(), it converts it to radians, so we will use degrees
        imu.initialize(parameters);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        shreyController = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        //YAW IS Z-AXIS
        //PITCH IS X-AX
        // IS
        //ROLL IS Y-AXIS
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        //i wanna see if the angles update in real time while the robot moves :D
        telemetry.addData("Yaw - Rotation About Z - Axis", robotOrientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch - Rotation about X - Axis", robotOrientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll - Rotation about Y - Axis", robotOrientation.getRoll(AngleUnit.DEGREES));

        double heading = robotOrientation.getYaw(AngleUnit.DEGREES);

        mecanumDrive.driveFieldCentric(
                //Joystick inputs range from -1.0D to 1.0D
                -shreyController.getLeftX(),
                -shreyController.getLeftY(),
                -shreyController.getRightX(),
                heading
        );

        telemetry.update();
    }
}


//Reminder: Robot Z points upwards to the ceiling.
// Robot Y points forward – whatever you decide is “forward” on your robot (which could be round!).
// Robot X points to the right side of the robot. Robot rotations follow the right-hand rule.
//copy pasted from doc xdd