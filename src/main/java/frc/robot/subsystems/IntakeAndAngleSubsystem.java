package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAndAngleSubsystem extends SubsystemBase{

    public static IntakeAndAngleSubsystem IAASubsystem;

    private final int angleMID = Constants.OperatorConstants.kAngleMotorID;
    private final int intakeMID = Constants.OperatorConstants.kIntakeMotorID;
    
    private CANSparkFlex intakeMotor = new CANSparkFlex(intakeMID, MotorType.kBrushless);
    private CANSparkFlex angleMotor = new CANSparkFlex(angleMID, MotorType.kBrushless);
    private AbsoluteEncoder encoder = angleMotor.getAbsoluteEncoder();
    private PIDController PID = new PIDController(0.0, 0.0, 0.0);

    public IntakeAndAngleSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(40);
        angleMotor.setSmartCurrentLimit(40);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * get the subsystem instance
     * @return returns the instance of this subsystem
     */
    public static IntakeAndAngleSubsystem getInstance() {
        if (IAASubsystem == null) {
            IAASubsystem = new IntakeAndAngleSubsystem();
        }
        return IAASubsystem;
    }

    /**
     * sets the setpoint to the given setpoint
     * Clamps to between range of absolute encoder (0-1)
     * @param setpoint the position you want it to go to
     */
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, 1);
        PID.setSetpoint(setpoint);
    }

    /**
     * sets the voltage given to the intake motor to the given amount
     * @param voltage amount of voltage to send
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
     * sets the voltage given to the intake motor to the given amount
     * @param voltage amount of voltage to send
     */
    public void setAngleVoltage(double voltage) {
        angleMotor.setVoltage(voltage);
    }

    /**
     * Update motor voltage based on the given position
     * @param pos current encoder position
     */
    public void updatePID(double pos) {
        double voltage = MathUtil.clamp(PID.calculate(pos), -2, 2);
        setAngleVoltage(voltage);
    }

    @Override
    public void periodic() {
        updatePID(encoder.getPosition());

        SmartDashboard.putNumber("Angle Encoder Pos", encoder.getPosition());
        SmartDashboard.putNumber("Angle Target Pos", PID.getSetpoint());
    }

}
