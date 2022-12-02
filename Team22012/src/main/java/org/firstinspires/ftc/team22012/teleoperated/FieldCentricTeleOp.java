package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorIMUOrthogonal;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
        imu.resetYaw();

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        shreyController = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        //YAW IS Z-AXIS
        //PITCH IS X-AXIS
        //ROLL IS Y-AXIS
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        //i wanna see if the angles update in real time while the robot moves :D
        double heading = robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw - Rotation About Z - Axis", robotOrientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch - Rotation about X - Axis", robotOrientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll - Rotation about Y - Axis", robotOrientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw - Rotatioin About Z - Axis in Radians", robotOrientation.getYaw(AngleUnit.RADIANS));
        //it says down here that the heading is imu.getAngularOrientation().firstAngle, therefore I will try to use BHI260 firstAngle too.
        //double heading = -imu.getAngularOrientation().firstAngle;
        //double heading = -imu.getRobotOrientation(
        //        AxesReference.INTRINSIC,
        //        AxesOrder.ZYX, //the first angle in the order is the yaw
/*DEG DIDNT WORK I WANNA TEST RADIANS */  // AngleUnit.RADIANS //mecanumDrive.driveFieldCentric() converts it to radians in the code, and requires it to be in deg
        //).firstAngle; //IT IS CONFIRMED THROUGH THE SS IN DISCORD THAT THE ANGLES ARE **AROUND** THE AXIS, THEREFORE AROUND THE Z AXIS WILL BE THE HEADING

        mecanumDrive.driveFieldCentric(
                //Joystick inputs range from -1.0D to 1.0D
                -shreyController.getLeftX()*0.45,
                -shreyController.getLeftY()*0.45,
                -shreyController.getRightX()*0.45,
                heading
        );

        telemetry.update();
    }
}


//Reminder: Robot Z points upwards to the ceiling.
// Robot Y points forward – whatever you decide is “forward” on your robot (which could be round!).
// Robot X points to the right side of the robot. Robot rotations follow the right-hand rule.
//copy pasted from doc xdd