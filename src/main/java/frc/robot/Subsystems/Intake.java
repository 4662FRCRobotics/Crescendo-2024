// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // one neo to spin intake and one to move intake
  // two limit switchs for in/out pos of intake, likly on the motor controller
  // itself
  // possiblity of limit switch to detect if the intake is full

  private CANSparkMax m_spinMotor;
  private CANSparkMax m_liftMotor;

  // based on what I have read Limit Switchs stop the motor on there own without
  // any code from me, further testing/research is needed
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  private SparkLimitSwitch m_isNoteIn;

  public Intake() {
    m_spinMotor = new CANSparkMax(CANIDConstants.kINTAKE_SPIN_MOTOR_ID, MotorType.kBrushless);
    m_liftMotor = new CANSparkMax(CANIDConstants.kINTAKE_LIFT_MOTOR_ID, MotorType.kBrushless);

    m_spinMotor.setIdleMode(IdleMode.kBrake);
    m_liftMotor.setIdleMode(IdleMode.kBrake);

    m_isNoteIn = m_spinMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_forwardLimit = m_liftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit = m_liftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
  }

  public void intakeNote() {
    m_spinMotor.set(1);
  }

  public void holdNote() {
    m_spinMotor.stopMotor();
  }

  public void dropNote() {
    m_spinMotor.set(-1);
  }

  public boolean isNoteIn(){
    return m_isNoteIn.isPressed();
  }

  // the reason for .2 is to test the limit switch before having it go fast
  public void extendIntake() {
    m_liftMotor.set(.2);
  }

  public void stopIntake() {
    m_liftMotor.stopMotor();
  }

  public void retractIntake() {
    m_liftMotor.set(-.2);
  }

  private boolean isIntakeExtended() {
    return m_forwardLimit.isPressed();
  }

  private boolean isIntakeRetracted() {
    return m_reverseLimit.isPressed();
  }
  public Command intakeIn() {
    return  Commands.run(() -> intakeNote(), this)
    .until(() -> isNoteIn())
    .andThen(() -> stopIntake(), this);
  }

  public Command intakeInLim() {
    return intakeIn()
    .unless(() -> isNoteIn());
  }

  public Command deployIntake() {
    return Commands.run(() -> extendIntake(), this)
    .until(() -> isIntakeExtended());
  }

  public Command deployIntakeEmpty() {
    return deployIntake()
    .unless(() -> isNoteIn() | isIntakeExtended());
  }

  public Command parkIntake() {
    return Commands.run(() -> retractIntake(), this)
    .until(() -> isIntakeRetracted());
  }
  public Command parkIntakeLoaded() {
    return parkIntake()
    .onlyIf(() -> isNoteIn());
  }


}
