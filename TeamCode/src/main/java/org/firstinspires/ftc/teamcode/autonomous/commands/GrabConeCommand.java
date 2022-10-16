package org.firstinspires.ftc.teamcode.autonomous.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.universal.subsystems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class GrabConeCommand extends CommandBase {

    private final ClawSubsystem claw;
    private final DoubleSupplier clawValue;

    public GrabConeCommand(ClawSubsystem claw, DoubleSupplier clawValue) {
        this.claw = claw;
        this.clawValue = clawValue;

        addRequirements(this.claw);
    }

    @Override
    public void execute() {
        claw.close(clawValue.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
