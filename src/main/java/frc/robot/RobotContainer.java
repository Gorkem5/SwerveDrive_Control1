package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Instantiate the SwerveSubsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Add data to SmartDashboard
    SmartDashboard.putData("SwerveSubsystem", swerveSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Bind the left stick Y axis to control the forward speed
    new Trigger(() -> Math.abs(m_driverController.getLeftY()) > 0.1).whileTrue(
      new RunCommand(() -> swerveSubsystem.drive(m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX()), swerveSubsystem)
    );
    // Bind the left stick Y axis to control the forward speed
    new Trigger(() -> Math.abs(m_driverController.getLeftY()) > 0.1).whileTrue(
      new RunCommand(() -> swerveSubsystem.drive(), swerveSubsystem)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return your autonomous command here
    return null;
  }
}