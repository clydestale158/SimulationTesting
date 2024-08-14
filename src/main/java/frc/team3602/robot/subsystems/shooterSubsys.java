package frc.team3602.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.team3602.robot.Constants.shooterConstants.*;

import java.util.function.DoubleSupplier;

public class shooterSubsys extends SubsystemBase {

/* Variables */
    //Motor controller variables
       public final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
       public final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

    //Encoders       
         private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
         private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();



/* the good stuff */
    public shooterSubsys() { 
        configShooterSubsys(); 
    }
   
     public Command runShooter(DoubleSupplier percentage){
        return runOnce ( () -> {
        bottomShooterMotor.set(percentage.getAsDouble());
        topShooterMotor.set(percentage.getAsDouble());
      } );
    }

    public Command bruhRunShooter(DoubleSupplier percentage){
        return runEnd(() ->{
            bottomShooterMotor.set(percentage.getAsDouble());
            topShooterMotor.set(percentage.getAsDouble());
        },
        () -> {
            stopShooters();
        }
        );
    }




    // public Command setBothShooters (DoubleSupplier percentage){
    //     return run(() -> {
    //         bottomShooterMotor.set(percentage.getAsDouble());
    //         topShooterMotor.set(percentage.getAsDouble());
    //      } );
    // }

    public Command stopShooters () {
        return run (() -> {
            bottomShooterMotor.set(0);
            topShooterMotor.set(0);
        });
    }




/* Shooter subsys configuration */
    public void configShooterSubsys () {
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        topShooterMotor.setSmartCurrentLimit(kShooterCurrentLimit);
        topShooterMotor.enableVoltageCompensation(topShooterMotor.getBusVoltage());
        topShooterMotor.setOpenLoopRampRate(0.000001);

        topShooterMotor.burnFlash();
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.setSmartCurrentLimit(kShooterCurrentLimit);
        bottomShooterMotor.enableVoltageCompensation(bottomShooterMotor.getBusVoltage());
        bottomShooterMotor.setOpenLoopRampRate(0.000001);

        bottomShooterMotor.burnFlash();
    }
}