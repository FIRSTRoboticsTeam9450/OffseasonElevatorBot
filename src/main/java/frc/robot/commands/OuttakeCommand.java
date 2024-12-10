package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

public class OuttakeCommand extends Command {

    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    private boolean dropped;
    // private boolean dontRun;

    private double voltage;

    public OuttakeCommand (double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        addRequirements(IAASubsystem);
        IAASubsystem.setIntakeVoltage(0);
        dropped = false;
        // dontRun = false;
        if (IAASubsystem.getLaserDistance() > 381) {
            // dontRun = true;
        }
    }

    @Override
    public void execute() {
        if (IAASubsystem.getLaserDistance() < 381 && !dropped) {
            IAASubsystem.setIntakeVoltage(-12);
        }
        if (IAASubsystem.getLaserDistance() > 381) {
            dropped = true;
        }
        if (voltage == 0.0) {
            IAASubsystem.setSetpoint(0.73);
        } else if (voltage == -12.0) {
            IAASubsystem.setSetpoint(0.485);
        }
        IAASubsystem.setIntakeVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return true;
        // if (timer.hasElapsed(Constants.kSpeedUpTime)) {

        // }
        // return (dropped || dontRun);
    }

    @Override
    public void end(boolean interrupted) {
        // IAASubsystem.setIntakeVoltage(0);
    }

}
