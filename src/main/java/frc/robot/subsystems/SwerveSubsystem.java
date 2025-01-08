package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveSubsystem extends SubsystemBase {

    public void drive(double forward, double strafe, double rotation) {

    }

    private final CommandXboxController m_driverController;

    private final double chassisWidth = Units.inchesToMeters(32); 
    private final double chassisLength = Units.inchesToMeters(32);

    // Define the locations of the modules on the robot.
    private final Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    private final Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
    private final Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
    private final Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

    // Define the kinematics object for the robot
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    );

    private final SwerveModule frontLeftModule = new SwerveModule(0, 1);
    private final SwerveModule frontRightModule = new SwerveModule(2, 3);
    private final SwerveModule backLeftModule = new SwerveModule(4, 5);
    private final SwerveModule backRightModule = new SwerveModule(6, 7);

    public SwerveSubsystem(CommandXboxController driverController) {
        this.m_driverController = driverController;
        System.out.println("SwerveSubsystem constructor");
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeftModule.setState(desiredStates[0]);
        frontRightModule.setState(desiredStates[1]);
        backLeftModule.setState(desiredStates[2]);
        backRightModule.setState(desiredStates[3]);
    }

    public void setChassisSpeed(ChassisSpeeds desired) {
        // Get the desired states for each module
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desired);

        // Set the desired states for each module
        setModuleStates(moduleStates);
    }

    public void drive() {
        double forward = -m_driverController.getLeftY();
        double strafe = m_driverController.getLeftX();
        double rotation = m_driverController.getRightX();

        // Convert the desired robot motion into swerve module states
        ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);

        // Set the desired states for each swerve module
        setModuleStates(desiredStates);
    }

    @Override
    public void periodic() {
        // Log the state of all swerve modules
        double[] loggingState = {
            frontLeftModule.getState().angle.getDegrees(),
            frontLeftModule.getState().speedMetersPerSecond,
            frontRightModule.getState().angle.getDegrees(),
            frontRightModule.getState().speedMetersPerSecond,
            backLeftModule.getState().angle.getDegrees(),
            backLeftModule.getState().speedMetersPerSecond,
            backRightModule.getState().angle.getDegrees(),
            backRightModule.getState().speedMetersPerSecond
        };

        // Sending data to the SmartDashboard
        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
    }
}

class SwerveModule {
    private final Talon driveMotor;
    private final Talon steeringMotor;
    private SwerveModuleState currentState;

    public SwerveModule(int driveMotorPort, int steeringMotorPort) {
        driveMotor = new Talon(driveMotorPort);
        steeringMotor = new Talon(steeringMotorPort);
        currentState = new SwerveModuleState();
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void setState(SwerveModuleState state) {
        this.currentState = state;
        driveMotor.set(state.speedMetersPerSecond);
        steeringMotor.set(convertAngleToMotorPosition(state.angle.getDegrees()));
    }

    private double convertAngleToMotorPosition(double angle) {
        // Implement the conversion logic here
        // This is a placeholder implementation
        return angle / 360.0;
    }
}