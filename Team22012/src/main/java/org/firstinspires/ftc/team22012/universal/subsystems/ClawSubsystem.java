package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {

    private final ServoEx leftServo, rightServo;

    //position of the claw: 0 is fully open and 300 is fully closed
    int position = 0;

    public ClawSubsystem(ServoEx leftServo, ServoEx rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        leftServo.setRange(0, 300, AngleUnit.DEGREES);
        rightServo.setRange(0, 300, AngleUnit.DEGREES);
        leftServo.setInverted(true);
        //these two set the servos to the open position.3
        leftServo.turnToAngle(0);
        rightServo.turnToAngle(0);
    }

    public double getLeftServoAngle() {
        return leftServo.getAngle(AngleUnit.DEGREES);
    }
    public double getRightServoAngle() {
        return rightServo.getAngle(AngleUnit.DEGREES);
    }

    public void closeFully() {
        leftServo.turnToAngle(40, AngleUnit.DEGREES);
        rightServo.turnToAngle( 40, AngleUnit.DEGREES);
    }
    public void closeForBeacon() {
        leftServo.turnToAngle(10, AngleUnit.DEGREES);
        rightServo.turnToAngle(10, AngleUnit.DEGREES);
    }

    public void openFully() {
        leftServo.turnToAngle(0, AngleUnit.DEGREES);
        rightServo.turnToAngle(0, AngleUnit.DEGREES);
    }

    public int getPosition() {
        return position;
    }

}
