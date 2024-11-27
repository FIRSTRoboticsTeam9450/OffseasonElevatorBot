package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ResetElevator extends Command {
    
    private Elevator elev = Elevator.getInstance();

    @Override
    public void initialize() {
        elev.setReset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
