/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class ShootAndDrive extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  //private final DriveTrain m_driveTrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final SwerveDrive m_swerveDrive;
  
  public ShootAndDrive(SwerveDrive swerveDrive, Feeder feeder, Shooter shooter) {
    m_swerveDrive = swerveDrive;
    m_shooter = shooter;
    m_feeder = feeder;
    
    addCommands(new SetShooterSpeed(m_shooter, ManualShooterSpeed.FAR_SETPOINT));
    addCommands(new ShootBall(m_feeder).withTimeout(1));
    addCommands(new SetShooterSpeed(m_shooter, 0.0));
    addCommands(new AutoSwerveJoystickCmd(m_swerveDrive, 0.5, 0.0, 0.0, false).withTimeout(2));
  }
}
