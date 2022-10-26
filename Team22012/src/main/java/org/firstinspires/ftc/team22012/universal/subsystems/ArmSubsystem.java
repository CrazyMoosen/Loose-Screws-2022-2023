package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {
    private Motor S1, S2;

    //the position of the linear slide
    int position;

    public ArmSubsystem(HardwareMap hardwareMap) {
        S1 = hardwareMap.get(Motor.class, "S1");
        S2 = hardwareMap.get(Motor.class, "S2");
        position = 0;

    }

    public void move(double value) {
    }

    public int getPosition() {
        return position;
    }
    public void completeExtend(){
        S1.setRunMode(Motor.RunMode.RawPower);
        S1.set(0.75);
        S2.setRunMode(Motor.RunMode.RawPower);
        S2.set(0.75);
    }

    public void armMove(double height){
        S1.setRunMode(Motor.RunMode.RawPower);
        S1.set(height);
        S2.setRunMode(Motor.RunMode.RawPower);
        S2.set(height);
    }

    public void completeRetract(){
        S1.setRunMode(Motor.RunMode.RawPower);
        S1.set(-0.75);
        S2.setRunMode(Motor.RunMode.RawPower);
        S2.set(-0.75);
    }

}
