package org.firstinspires.ftc.team22012.universal.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team22012.universal.subsystems.ClawSubsystem;

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
