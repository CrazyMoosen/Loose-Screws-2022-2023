package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSubsystem extends SubsystemBase {
    private ServoEx servo1, servo2;
    HardwareMap hardwareMap;
    private double rotatedby1;
    private double rotatedby2;
    //initialize claw
    public ClawSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
         servo1 = new SimpleServo(
                hardwareMap, "servo1", 0, 300
        );
         servo2 = new SimpleServo(
                hardwareMap, "servo2", 0, 300
        );
    }
    //function that closes the gripper, parameter is the value taken to grab the cone
    public void close(double value) {
        servo1.setInverted(true);
        if (rotatedby1<=90) {
            servo1.rotateByAngle(value);
            rotatedby1 = rotatedby1+value;
        }
        if (rotatedby2<=90) {
            servo2.rotateByAngle(value);
            rotatedby2 = rotatedby2+value;
        }
    }

    //function that releases the cone from the grip of the gripper
    public void release(double value) {
        if (rotatedby1<=90 && rotatedby1>=-1) {
            servo1.rotateByAngle(-value);
            rotatedby1 = rotatedby1-value;
        }
        if (rotatedby2<=90 && rotatedby2>=-1) {
            servo2.rotateByAngle(-value);
            rotatedby2 = rotatedby2-value;
        }
    }

}
