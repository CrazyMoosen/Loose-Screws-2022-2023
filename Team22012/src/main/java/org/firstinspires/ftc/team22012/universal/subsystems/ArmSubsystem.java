package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {

    Motor linearSlideMotor;
    //the position of the linear slide
    int position;

    public ArmSubsystem(HardwareMap hardwareMap) {

        linearSlideMotor = new Motor(hardwareMap, "linearSlideMotor");
        position = 0;

    }

    public void move(double value) {
    }

    public int getPosition() {
        return position;
    }

}
