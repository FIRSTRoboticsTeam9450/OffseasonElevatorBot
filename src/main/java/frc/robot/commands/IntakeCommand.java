package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

public class IntakeCommand extends Command{

    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    //The distance for the LaserCan to say it sees something
    //381mm = 15"
    private double triggerDistance = 381;

    @Override
    public void initialize() {
        addRequirements(IAASubsystem);
        if (IAASubsystem.getLaserDistance() > triggerDistance) {
            IAASubsystem.setIntakeVoltage(1);
        }
    }

    @Override
    public void execute() {
        if (IAASubsystem.getLaserDistance() > triggerDistance) {
            IAASubsystem.setIntakeVoltage(1);
        }
    }

    @Override
    public boolean isFinished() {
        return IAASubsystem.getLaserDistance() < triggerDistance;
    }

    @Override
    public void end(boolean interrupted) {
        IAASubsystem.setIntakeVoltage(0);
    }

}