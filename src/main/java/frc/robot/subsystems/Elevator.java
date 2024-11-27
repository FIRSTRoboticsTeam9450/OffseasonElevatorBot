package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private static Elevator elevator;

    private final int m1ID = Constants.OperatorConstants.kElevatorMotor1ID;
    private final int m2ID = Constants.OperatorConstants.kElevatorMotor2ID;

    private CANSparkFlex motor1 = new CANSparkFlex(m1ID, MotorType.kBrushless);
    private CANSparkFlex motor2 = new CANSparkFlex(m2ID, MotorType.kBrushless);
    private RelativeEncoder encoder = motor1.getEncoder();
    private SparkLimitSwitch limitSwitch = motor1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private PIDController PID = new PIDController(1, 0, 0.02);
    boolean reseting = false;
    boolean runPID = true;

    public Elevator() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor1.setInverted(false);
        motor2.setInverted(false);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        encoder.setPosition(0);

        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);

    }

    /**
     * Return a static instance of Elevator
     * @return Instance of Elevator */
    public static Elevator getInstance() {
        if (elevator == null) {
            elevator = new Elevator();
        }
        return elevator;
    }

    /**
     * Sets the motor's voltage to the given voltage
     * @param voltage voltage to set
     */
    public void setVoltage(double voltage) {
        motor1.setVoltage(voltage);
        motor2.setVoltage(voltage);
    }

    /**
     * Sets the position it'll go to to the given setpoint
     * @param setpoint point to go to (0 to 19)
     */
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, 19);
        PID.setSetpoint(setpoint);
    }

    /**
     * Update motor voltage based on the given position
     * @param position current encoder position
     */
    public void updatePID(double position) {
        double voltage = MathUtil.clamp(PID.calculate(position), -6, 12);
        setVoltage(voltage);
    }

    @Override
    public void periodic() {
        if (!reseting && runPID) {
            updatePID(encoder.getPosition());
        }

        SmartDashboard.putBoolean("Limit Switch", limitSwitch.isPressed());
        SmartDashboard.putNumber("Encoder Pos", encoder.getPosition());
        SmartDashboard.putNumber("Target Pos", PID.getSetpoint());
        
        if (limitSwitch.isPressed()) {
            encoder.setPosition(-0.5);
            reseting = false;
        }
    }

    public void setReset() {
        reseting = true;
        setVoltage(-1);
    }

}
