package org.firstinspires.ftc.teamcode.autonomous.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.universal.subsystems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ReleaseConeCommand extends CommandBase {
    private final ClawSubsystem claw;

    public ReleaseConeCommand(ClawSubsystem claw) {
        this.claw = claw;

        addRequirements(this.claw);
    }

    @Override
    public void execute() {
        claw.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
