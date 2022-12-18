package org.firstinspires.ftc.team22012.universal.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem extends SubsystemBase {
    private Motor linearSlideMotor;

    //the position of the linear slide
    double position;
    ElapsedTime elapsedTime = new ElapsedTime();

    public ArmSubsystem(Motor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
        position = 0;

    }
    public void moveAuto() {
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000) {
            moveup();
        }
        stop();
    }
    public void moveup() {
        linearSlideMotor.set(1);
    }
    public void movedown() {
        linearSlideMotor.set(-0.5);
    }
    public void stop() {
        linearSlideMotor.set(0);
    }
    public double getPosition() {
        return position;
    }

}
