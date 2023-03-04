package org.firstinspires.ftc.team22012.universal.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {

    private final Servo clawServo;

    //position of the claw: 0 is fully open and 300 is fully closed
    int position = 0;

    public ClawSubsystem(Servo clawServo) {
        this.clawServo = clawServo;
        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.getController().pwmEnable();
        clawServo.setPosition(0.3);
    }
    public void closeFully() {
        clawServo.setPosition(0.7); //0 is minimum position and 1 is max position
    }

    public void openFully() {
        clawServo.setPosition(0);
    }

    public int getPosition() {
        return position;
    }

}