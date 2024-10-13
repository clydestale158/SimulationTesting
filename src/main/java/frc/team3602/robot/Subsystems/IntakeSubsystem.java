/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import monologue.Logged;
// import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase  {
  // Motor controllers
  //@Log
  private double intakeOut;

  //@Log
  private double intakeVolts;

  public CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);

  // Sensors
  //@Log
  public boolean hasNote = true;

  private final DigitalInput sensor1 = new DigitalInput(3);
  private final DigitalInput sensor2 = new DigitalInput(1);

  public final Spark blinkin = new Spark(0);

  public IntakeSubsystem() {
    configIntakeSubsys();
  }

 public boolean getSensor1() {
    return sensor1.get();
  }

  public boolean getSensor2() {
    return sensor2.get();
  }

  public Command runIntake(DoubleSupplier percentage) {
    return runEnd(() -> {
      intakeMotor.set(percentage.getAsDouble());
    },
    () -> {
      intakeMotor.set(0.0);
    });
  }

  public Command stopIntake() {
    return runOnce(() -> intakeMotor.set(0.0));
  }

  @Override
  public void periodic() {
    intakeOut = intakeMotor.getAppliedOutput();

    intakeVolts = (intakeMotor.getAppliedOutput() * 12);

    hasNote = getSensor1() || getSensor2();

    // if(hasNote) blinkin.setVoltage(0.99);
    // else blinkin.setVoltage(0.77);
  }

  private void configIntakeSubsys() {
    // Intake motor config
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(kIntakeMotorCurrentLimit);
    intakeMotor.enableVoltageCompensation(intakeMotor.getBusVoltage());
    intakeMotor.setOpenLoopRampRate(0.000001);

    intakeMotor.burnFlash();
  }
}