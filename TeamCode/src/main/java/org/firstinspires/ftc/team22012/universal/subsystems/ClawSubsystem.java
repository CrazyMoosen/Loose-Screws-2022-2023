package org.firstinspires.ftc.team22012.universal.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem {

    private final Servo leftServo, rightServo;

    //position of the claw: 0 is fully open and 300 is fully closed
    int position = 0;

    public ClawSubsystem(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        leftServo.scaleRange(0, 300);
        rightServo.scaleRange(0, 300);
        leftServo.setDirection(Servo.Direction.REVERSE);

        //these two set the servos to the open position.3
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public double getLeftServoAngle() {
        return leftServo.getPosition();
    }
    public double getRightServoAngle() {
        return rightServo.getPosition();
    }

    public void closeFully() {
        leftServo.setPosition(40/300d);
        rightServo.setPosition(40/300d);
    }

    public void openFully() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public int getPosition() {
        return position;
    }

}
