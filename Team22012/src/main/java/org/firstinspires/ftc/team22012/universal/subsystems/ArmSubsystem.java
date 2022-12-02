package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private Motor linearSlideMotor1;

    //the position of the linear slide
    double position;

    public ArmSubsystem(HardwareMap hardwareMap) {
        linearSlideMotor1 = hardwareMap.get(Motor.class, "S1");
        position = 0;

    }

    public void move(double value) {

    }

    public double getPosition() {
        return position;
    }

    public void completeExtend(){

    }

    public void completeRetract(){

    }

}
