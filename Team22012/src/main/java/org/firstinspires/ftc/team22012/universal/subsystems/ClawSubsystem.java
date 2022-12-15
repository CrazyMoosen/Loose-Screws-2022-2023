package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {
    //trust this is all theoretical the probability this works is 5%

    private final ServoEx leftServo, rightServo;

    //position of the claw: 0 is fully open and 180 is fully closed
    int position = 0;

    public ClawSubsystem(ServoEx leftServo, ServoEx rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        leftServo.setRange(0, 180, AngleUnit.DEGREES);
        rightServo.setRange(0, 180, AngleUnit.DEGREES);

        //these two set the servos to the open position
        leftServo.turnToAngle(0);
        rightServo.turnToAngle(0);
    }

    public double getLeftServoAngle() {
        return leftServo.getAngle(AngleUnit.DEGREES);
    }
    public double getRightServoAngle() {
        return rightServo.getAngle(AngleUnit.DEGREES);
    }

    public void move(int endPos) {
        //close the claw
        if (endPos <= 180 && endPos >= 0) {
            leftServo.turnToAngle(endPos, AngleUnit.DEGREES);
            rightServo.turnToAngle(endPos, AngleUnit.DEGREES);
            position = endPos;
        }
    }

    public int getPosition() {
        return position;
    }

}
