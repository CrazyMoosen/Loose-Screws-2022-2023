package org.firstinspires.ftc.team22012.universal.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem extends SubsystemBase {
    private final Motor linearSlideMotor;

    //the position of the linear slide
    ElapsedTime elapsedTime = new ElapsedTime();

    public ArmSubsystem(Motor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
        this.linearSlideMotor.setInverted(true);

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
        linearSlideMotor.set(-1);
    } //for testing purposes i put 1 we can change to slower later

    //gentlyMoveDown is not needed as gravity is cool
    public void gentlyMoveUp() {
        linearSlideMotor.set(0.4);
    }
    public void stop() {
        linearSlideMotor.set(0);
    }

}
