package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private Motor linearSlideMotor;

    //the position of the linear slide
    double position;

    public ArmSubsystem(Motor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
        position = 0;

    }

    public void moveup() {
        linearSlideMotor.set(0.3);
    }
    public void movedown() {
        linearSlideMotor.set(-0.3);
    }
    public void stop() {
        linearSlideMotor.set(0);
    }
    public double getPosition() {
        return position;
    }

}
