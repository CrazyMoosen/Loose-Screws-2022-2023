package org.firstinspires.ftc.team22012.universal.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem extends SubsystemBase {
    private final Motor linearSlideMotor;
    public double finalPosition = 5.6;

    //the position of the linear slide
    ElapsedTime elapsedTime = new ElapsedTime();

    public ArmSubsystem(Motor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
        this.linearSlideMotor.setInverted(true);
        //this.linearSlideMotor.setRunMode(Motor.RunMode.PositionControl);
    }
    public void moveAuto() {
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000) {
            moveUp();
        }
        stop();
    }

    public void stallarm() {
        linearSlideMotor.set(0.15);
    }

//    public void move(double value) {
//
//    }

    public void moveUp() {
        linearSlideMotor.set(1);
    }
    public void moveDown() { linearSlideMotor.set(-1); }
    public void stop(){linearSlideMotor.set(0);}
    //gentlyMoveDown is not needed as gravity is cool
    public void gentlyMoveUp() {
        linearSlideMotor.set(0.4);
    }

}
