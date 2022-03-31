/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberHigh extends SubsystemBase {
  private static final int CLIMBERHIGH1_CAN_ID = 15;
  private static final int CLIMBERHIGH2_CAN_ID = 16;
  private static final int RELEASEMOTOR1_CAN_ID = 18; //Pigeon is 17 I think
  private static final int RELEASEMOTOR2_CAN_ID = 19;

  TalonFX climberHighMotor1;
  TalonFX climberHighMotor2;
  static CANSparkMax releaseMotor1;
  static CANSparkMax releaseMotor2;
  
  public ClimberHigh() {
    climberHighMotor1 = new TalonFX(CLIMBERHIGH1_CAN_ID);
    climberHighMotor1.setNeutralMode(NeutralMode.Brake);
    climberHighMotor2 = new TalonFX(CLIMBERHIGH2_CAN_ID);
    climberHighMotor2.setNeutralMode(NeutralMode.Brake);
    
    releaseMotor1 = new CANSparkMax(RELEASEMOTOR1_CAN_ID, MotorType.kBrushless);
    releaseMotor1.setIdleMode(IdleMode.kBrake);
    releaseMotor1.setSmartCurrentLimit(20);
    releaseMotor2 = new CANSparkMax(RELEASEMOTOR2_CAN_ID, MotorType.kBrushless);
    releaseMotor2.setIdleMode(IdleMode.kBrake);
    releaseMotor2.setSmartCurrentLimit(20);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    //If one climber or the other is spinning the wrong direction, change speed to -speed below
    climberHighMotor1.set(ControlMode.PercentOutput, speed);
    climberHighMotor2.set(ControlMode.PercentOutput, speed);
  }

  public static void release() {
    //flip signs to correct rotation direction if needed
    releaseMotor1.set(0.25);
    releaseMotor2.set(-0.25);
  }

}