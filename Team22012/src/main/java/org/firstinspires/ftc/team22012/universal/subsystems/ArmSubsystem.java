package org.firstinspires.ftc.team22012.universal.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {
    private Motor S1, S2;

    public ArmSubsystem(HardwareMap hardwareMap) {

    }
    public void completeExtend(){
        S1.setRunMode(Motor.RunMode.RawPower);
        S1.set(0.75);
        S2.setRunMode(Motor.RunMode.RawPower);
        S2.set(0.75);
    }
    public void ArmMove(double height){
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
