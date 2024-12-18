package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private static Elevator elevator;

    /* ----- Motor Stuff ----- */
    // private CANSparkFlex motor1 = new CANSparkFlex(Constants.kElevatorMotor1ID, MotorType.kBrushless); // PDH ports 2, 7
    // private CANSparkFlex motor2 = new CANSparkFlex(Constants.kElevatorMotor2ID, MotorType.kBrushless);
    // private RelativeEncoder encoder = motor1.getEncoder();
    TalonFX motor1 = new TalonFX(Constants.kElevatorMotor1ID, "CantDrive");
    TalonFX motor2 = new TalonFX(Constants.kElevatorMotor2ID, "CantDrive");

    /* ----- Other Stuff ----- */
    // private SparkLimitSwitch limitSwitch = motor1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private PIDController PID = new PIDController(0.7, 0, 0.0);
    boolean reseting = false;
    boolean runPID = true;

    double rampTime = 0.5;

    Timer timer = new Timer();
    

    public Elevator() {
        //Motor Settings
        // motor1.restoreFactoryDefaults();
        // motor2.restoreFactoryDefaults();
        motor1.setInverted(false);
        motor2.setInverted(false);
        // motor1.setSmartCurrentLimit(80);
        // motor2.setSmartCurrentLimit(80);

        motor1.setPosition(0);

        motor1.setNeutralMode(NeutralModeValue.Brake);
        motor2.setNeutralMode(NeutralModeValue.Brake);

        timer.restart();

    }

    /* ----- Updaters ----- */

    /**
     * Update motor voltage based on the given position
     * @param position current encoder position
     */
    public void updatePID(double position) {
        double voltage = MathUtil.clamp(PID.calculate(position), -1, 9);
        voltage *= timer.get() >= rampTime ? 1 : timer.get() /  rampTime;
        setVoltage(voltage);
        SmartDashboard.putNumber("Elevator Voltage", voltage);
        Logger.recordOutput("elevator/pos", position);
        Logger.recordOutput("elevator/setpoint", PID.getSetpoint());
        Logger.recordOutput("elevator/motorVoltages", voltage);
    }

    // public void reset() {
    //     reseting = true;
    //     motor1.setVoltage(-1);
    //     motor2.setVoltage(-1);
    //     double difference = motor1.getDifferentialDifferencePosition().getValueAsDouble();
    //     if (Math.abs(difference) < 10) {
    //         motor1.setVoltage(0);
    //         motor2.setVoltage(0);
    //     }
    // }

    @Override
    public void periodic() {
        if (!reseting && runPID) {
            updatePID(motor1.getPosition().getValueAsDouble());
        }

        // SmartDashboard.putBoolean("Limit Switch", limitSwitch.isPressed());
        SmartDashboard.putNumber("Encoder Pos", motor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Pos", PID.getSetpoint());
        
        // if (limitSwitch.isPressed()) {
        //     encoder.setPosition(-0.5);
        //     reseting = false;
        // }
    }

    /* ----- Setters and Getters ----- */

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
        timer.restart();
        setpoint = MathUtil.clamp(setpoint, 0, 23); //high: 27
        PID.setSetpoint(setpoint);
    }

    public void setReset() {
        reseting = true;
        setVoltage(-1);
        double allowedDifference = 0.025;
        double pos1 = motor1.getPosition().getValueAsDouble();
        double pos2 = pos1;
        pos1 = motor1.getPosition().getValueAsDouble();
        if (Math.max(pos1, pos2) - Math.min(pos1, pos2) < allowedDifference) {
            reseting = false;
            setVoltage(0);
        }
    }

    /**
     * will return the position of the elevator
     * @return
     */
    public double getPosition() {
        return motor1.getPosition().getValueAsDouble();
    }

}
