// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AngleCommand;
import frc.robot.commands.HoldingCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.SetPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private double MaxSpeed = 0.35; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 0.3 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static boolean holding = true;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Elevator elev = Elevator.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    m_driverController.a().onTrue(new SetPosition(0));
    m_driverController.y().onTrue(new SetPosition(19));
    m_driverController.rightTrigger().whileTrue(new IntakeCommand(12));
    m_driverController.rightTrigger().onFalse(new IntakeCommand(0));
    m_driverController.leftTrigger().onTrue(new AngleCommand(0.4).andThen(new OuttakeCommand(-12)));
    m_driverController.leftTrigger().whileFalse(new OuttakeCommand(0).andThen(new AngleCommand(.725)));

    m_driverController.b().onTrue(new AngleCommand(0.73));
    m_driverController.x().onTrue(new AngleCommand(0.4));

    m_driverController.povDown().onTrue(new HoldingCommand(!holding));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
