package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAndAngleSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class AngleCommand extends Command{
    private double pos;
    private IntakeAndAngleSubsystem IAASubsystem = IntakeAndAngleSubsystem.getInstance();

    /**
     * uses absolute encoder
     * Given pos is clamped between .482 and .73
     * NEVER go < .38, you will bang into the elevator system if it is raised
     * @param pos the pos you want it to go to 
     */
    public AngleCommand(double pos) {
        pos = MathUtil.clamp(pos, .482, .725);
        this.pos = pos;
    }

    @Override
    public void initialize() {
        IAASubsystem.setSetpoint(pos);
    }

    @Override
    public boolean isFinished() {
        return IAASubsystem.getPosition() < pos + 0.05 && IAASubsystem.getPosition() > pos - 0.05 ? true : false;
    }

}
