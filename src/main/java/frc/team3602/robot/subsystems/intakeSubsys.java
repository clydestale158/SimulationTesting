package frc.team3602.robot.subsystems;

import static frc.team3602.robot.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class intakeSubsys extends SubsystemBase {
  /*variables */
    //motor
    private final CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);
    //sensors
    public boolean hasNote = true;

    public final DigitalInput beamSensor = new DigitalInput(kBeamSensorId);
    public final DigitalInput colorSensor = new DigitalInput(kColorSensorId);

  /*the good stuff */
    public intakeSubsys() {
        configIntakeSubsys();
    }

    public boolean getSensors() {
        return (beamSensor.get() || colorSensor.get());
    }

    // commands to refer to 
    public Command runIntake(DoubleSupplier percentage){
        return runEnd(() -> {
            intakeMotor.set(percentage.getAsDouble());
        },
        () -> {
            intakeMotor.set(0.0);
        });
    }

    // @Override
    // public void periodic() {
    //     hasNote = getSensors();
    // }

    public void configIntakeSubsys() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(kIntakeMotorCurrentLimit);
        intakeMotor.enableVoltageCompensation(intakeMotor.getBusVoltage());
        intakeMotor.setOpenLoopRampRate(0.000001);

        intakeMotor.burnFlash();
    }
}