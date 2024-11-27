package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetPosition extends Command{
    
    private double pos;
    private Elevator elev = Elevator.getInstance();

    /**
     * elephant
     * @param pos the pos where you want it to go
     */
    public SetPosition(double pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        elev.setSetpoint(pos);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

}
