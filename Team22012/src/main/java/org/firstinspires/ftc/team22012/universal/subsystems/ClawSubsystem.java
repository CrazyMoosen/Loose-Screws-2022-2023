package org.firstinspires.ftc.team22012.universal.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSubsystem extends SubsystemBase {

    HardwareMap hardwareMap;

    //initialize claw
    public ClawSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    //function that closes the gripper, parameter is the value taken to grab the cone
    public void close(double value) {

    }

    //function that releases the cone from the grip of the gripper
    public void release() {

    }

}
