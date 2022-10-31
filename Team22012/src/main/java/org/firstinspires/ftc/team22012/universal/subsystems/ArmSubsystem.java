package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {
    private Motor linearSlideMotor1, linearSlideMotor2;

    //the position of the linear slide
    int position;

    public ArmSubsystem(HardwareMap hardwareMap) {
        linearSlideMotor1 = hardwareMap.get(Motor.class, "S1");
        linearSlideMotor2 = hardwareMap.get(Motor.class, "S2");
        position = 0;

    }

    public void move(double value) {

    }

    public int getPosition() {
        return position;
    }

    public void completeExtend(){

    }

    public void armMove(double height){

    }

    public void completeRetract(){

    }

}
