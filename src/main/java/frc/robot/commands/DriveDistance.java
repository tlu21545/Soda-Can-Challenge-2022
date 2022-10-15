package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class DriveDistance extends CommandBase{
  private final Drivetrain m_drivetrain;
  private final double m_speed;
  private final double m_distance;
  private final double kP_drive = DriveConstants.kP_drive;

  private double drive_error;
  private double m_rotation;

    /**
    * @param speed The speed at which the robot will drive
    * @param inches The number of inches the robot will drive
    * @param drivetrain The drivetrain subsystem on which this command will run
    */
    public DriveDistance(double speed, double inches, Drivetrain drivetrain) {
        m_speed = speed;
        m_distance = inches;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.arcadeDrive(0, 0);
        m_drivetrain.resetEncoders();
        drive_error = 0;
        m_rotation = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive_error = m_drivetrain.getDriveError();
        m_rotation = -kP_drive * drive_error;
        m_drivetrain.arcadeDrive(m_speed, m_rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(0, 0);
        m_drivetrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_drivetrain.getAverageDistanceInch()) >= m_distance;
    }
}
