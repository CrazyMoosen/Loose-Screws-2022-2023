package org.firstinspires.ftc.team22012.teleoperated;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FieldCentric", group = "Drive Modes")
public class FieldCentricTeleOp extends OpMode {

    private Motor fL, fR, bL, bR;
    private BNO055IMU imu;
    private MecanumDrive mecanumDrive;
    private GamepadEx shreyController;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        //need FTC Version 8.2 to get it.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        mecanumDrive = new MecanumDrive(fL, fR, bL, bR);
        shreyController = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        double heading = -imu.getAngularOrientation().firstAngle;
        mecanumDrive.driveFieldCentric(
                shreyController.getLeftX(),
                shreyController.getLeftY(),
                shreyController.getRightX(),
                heading
        );
    }
}
