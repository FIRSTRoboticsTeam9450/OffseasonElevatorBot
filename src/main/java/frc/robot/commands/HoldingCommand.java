package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

/**
 * Temporary, using for while the intake system 
 */
public class HoldingCommand extends Command{

    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    private boolean holding;
    
    public HoldingCommand (boolean temp) {
        holding = temp;
        RobotContainer.holding = !holding;
    }

    @Override
    public void initialize() {
        if (holding) {
            IAASubsystem.setIntakeVoltage(1.5);
        } else {
            IAASubsystem.setIntakeVoltage(0);
        }
    }

}
