package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private Motor linearSlideMotor1;

    //the position of the linear slide
    double position;

//    public ArmSubsystem(HardwareMap hardwareMap) {
//        linearSlideMotor1 = hardwareMap.get(Motor.class, "linearSlideMotor1");
//        position = 0;
//
//    }
//
//    public void moveup() {
//        linearSlideMotor1.set(0.3);
//    }
//    public void movedown() {
//        linearSlideMotor1.set(-0.3);
//    }
//    public void stop() {
//        linearSlideMotor1.set(0);
//    }
//    public double getPosition() {
//        return position;
//    }

}
