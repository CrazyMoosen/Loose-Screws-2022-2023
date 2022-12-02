package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;
    private final BHI260IMU imu;
    private final MecanumDriveOdometry m_odometry;
    MecanumDriveKinematics m_kinematics;
    ElapsedTime elapsedTime = new ElapsedTime();

    double ticksPerMeter = 1193.66207;

    Motor fL, fR, bL, bR;
    public DriveSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, //Orthogonal #9 in the docs
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        //this here set it to radians before, however in mecanumDrive.driveFieldCentric(), it converts it to radians, so we will use degrees
        imu.initialize(parameters);
        imu.resetYaw();

        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        m_odometry = new MecanumDriveOdometry(m_kinematics,
                Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES)), new Pose2d(5.0, 13.5, new Rotation2d()));
        elapsedTime.reset();
    }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void periodic() {

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        //m_odometry.updateWithTime(elapsedTime.seconds(), Rotation2d.fromDegrees(robotOrientation.getYaw(AngleUnit.DEGREES)), fL);
    }
}
