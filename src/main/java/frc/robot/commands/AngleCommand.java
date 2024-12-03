package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;
import edu.wpi.first.math.MathUtil;

public class AngleCommand extends Command{
    private double pos;
    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    /**
     * uses absolute encoder
     * Given pos is clamped between .4 and .71
     * @param pos the pos you want it to go to 
     */
    public AngleCommand(double pos) {
        pos = MathUtil.clamp(pos, .4, .71);
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
