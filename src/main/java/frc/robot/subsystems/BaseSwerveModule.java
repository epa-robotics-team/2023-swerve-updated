package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

abstract public class BaseSwerveModule {
  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract void setDesiredState(SwerveModuleState desiredState);

  public abstract void resetEncoders();
}