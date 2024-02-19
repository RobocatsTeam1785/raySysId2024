// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurnMotor extends SubsystemBase {
  /** Creates a new TurnMotor. */
  CANSparkMax motor = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

  RelativeEncoder encoder = motor.getEncoder();
  // encoder.setVelocityConversionFactor(2 * Math.pi / 60);
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism( 
    (Measure<Voltage> volts) -> {
    motor.setVoltage(volts.in(Volts));
    }, log -> {
      log.motor("drive-left").voltage(m_appliedVoltage.mut_replace(motor.get() * RobotController.getBatteryVoltage(),Volts))
      .linearPosition(m_distance.mut_replace(encoder.getPosition() , Meters))
      .linearVelocity(m_velocity.mut_replace(encoder.getVelocity(), MetersPerSecond));
      System.out.println("volts: " + motor.get() * RobotController.getBatteryVoltage());
    }
    , this));
  public TurnMotor() {
//turn motor ID:
    // setTurnMotor(Measure<Voltage>)
  }

  public Command sysIdQuasistaticForward() {
    return routine.quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  public Command sysIdQuasistaticBackward() {
    return routine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return routine.dynamic(SysIdRoutine.Direction.kForward);
  }

    public Command sysIdDynamicBackward() {
    return routine.dynamic(SysIdRoutine.Direction.kReverse);
  }
  // public void setTurnMotor(Measure<Voltage> volts)
  // {
  //   motor.setVoltage(volts.in(Volts));
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
