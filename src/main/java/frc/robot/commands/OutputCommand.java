package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

public class OutputCommand extends Command {

    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    private boolean dropped;
    private boolean dontRun;

    @Override
    public void initialize() {
        addRequirements(IAASubsystem);
        IAASubsystem.setIntakeVoltage(0);
        dropped = false;
        dontRun = false;
        if (IAASubsystem.getLaserDistance() > 381) {
            dontRun = true;
        }
    }

    @Override
    public void execute() {
        if (IAASubsystem.getLaserDistance() < 381 && !dropped) {
            IAASubsystem.setIntakeVoltage(-1);
        }
        if (IAASubsystem.getLaserDistance() > 381) {
            dropped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (dropped || dontRun);
    }

    @Override
    public void end(boolean interrupted) {
        IAASubsystem.setIntakeVoltage(0);
    }

}
