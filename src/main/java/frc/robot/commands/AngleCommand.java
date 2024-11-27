package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

public class AngleCommand extends Command{
    private double pos;
    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    /**
     * IDK what to put here
     * @param pos the pos you want it to go to 
     */
    public AngleCommand(double pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        IAASubsystem.setSetpoint(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
