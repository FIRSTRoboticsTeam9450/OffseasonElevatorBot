package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAndAngleSubsystem extends SubsystemBase{

    /* ----- Subsystem Instance ----- */
    public static IntakeAndAngleSubsystem IAASubsystem;

    /* ----- Laser Can ----- */
    private LaserCan laserCan;
    private LaserCan.Measurement measurement;
    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    MedianFilter medianDistance = new MedianFilter(3);
    double laserDistance;
    
    /* ----- Motor ----- */
    private CANSparkFlex intakeMotor = new CANSparkFlex(Constants.kIntakeMotorID, MotorType.kBrushless);
    private CANSparkFlex angleMotor = new CANSparkFlex(Constants.kAngleMotorID, MotorType.kBrushless);
    private AbsoluteEncoder encoder = angleMotor.getAbsoluteEncoder();
    private PIDController PID = new PIDController(17, 0.0, 0.8);

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.25, 0.1);

    private double setpoint;
    
    // private ProfiledPIDController pidTest = new ProfiledPIDController(17, 0, 0.8, constraints);

    /**
     * sets Intake and Anlge motor to certain settings
     * Settings:
     * restoreFActoryDefaults()
     * setSmartCurrentLimit(40)
     * Idle mode to brake
     */
    public IntakeAndAngleSubsystem() {

        //LaserCan settings
        laserCan = new LaserCan(Constants.kLaserCanID);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.LONG);

            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Error during Laser Can Configuration: " + e);
        }

        measurement = laserCan.getMeasurement();

        //Motor Settings
        intakeMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(40);
        angleMotor.setSmartCurrentLimit(40);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);

        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        //Intake system starting position
        PID.setSetpoint(0.725);
    }

    /* ----- Updaters ----- */

    /**
     * Updates the median distance of the laser can for the past 3 checks
     */
    public void updateLasers() {
        try {
            measurement = laserCan.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                laserDistance = medianDistance.calculate(measurement.distance_mm);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
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
        double encoderPos = encoder.getPosition();
        updatePID(encoderPos);
        updateLasers();

        SmartDashboard.putNumber("Angle Encoder Pos", encoderPos);
        SmartDashboard.putNumber("Angle Target Pos", PID.getSetpoint());
        SmartDashboard.putNumber("Laser Distance", laserDistance);
        Logger.recordOutput("IntakeSubsystem/Angle at setpoint", encoder.getPosition() < setpoint + 0.05 && encoder.getPosition() > setpoint - 0.05 ? true : false);
    }

    /* ----- Setters and Getters ----- */

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
     * returns the distance of the laser from the laser can
     * @return distance of laser
     */
    public double getLaserDistance() {
        return laserDistance;
    }

    /**
     * sets the setpoint to the given setpoint
     * Clamps to between range of absolute encoder (0.482-0.73)
     * @param setpoint the position you want it to go to
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        setpoint = MathUtil.clamp(setpoint, 0.482, 0.73);
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

    public double getPosition() {
        return encoder.getPosition();
    }
    
}
