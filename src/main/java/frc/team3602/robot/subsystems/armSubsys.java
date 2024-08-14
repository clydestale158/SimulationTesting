package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3602.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

public class armSubsys extends SubsystemBase {
 /*variables */
    //motors
    private final CANSparkMax leftMotor = new CANSparkMax(kLeftArmId, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(kRightArmId, MotorType.kBrushless);

    //encoders
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    //controls
    private double rightTarget, leftTarget;
    private final PIDController rightController = new PIDController(kP, kI, kD);
    private final PIDController leftController = new PIDController(kP, kI, kD);

    private final ArmFeedforward rightFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    private final ArmFeedforward leftFeedforward = new ArmFeedforward(kS, kG, kV, kA);


 /*the good stuff */
    public armSubsys() {
        rightTarget = kRetractedHeight;
        leftTarget = kRetractedHeight;

        resetEncoders();
        configArmSubsys();
    }

    public void resetEncoders() {
        rightEncoder.setPosition(kExtendedHeight);
        leftEncoder.setPosition(kExtendedHeight);
    }

    public Command holdHeights() {
        return runOnce(() -> {
            rightMotor.setVoltage(getRightEffort());
            leftMotor.setVoltage(getLeftEffort());
        });
    }

    public  Command setHeight(DoubleSupplier heightInches) {
        return runOnce(() -> {
            rightTarget = heightInches.getAsDouble();
            leftTarget = heightInches.getAsDouble();
        });
    }



    private double getLeftEncoder(){
        return leftEncoder.getPosition();
    }

    private double getLeftEffort(){
        var ffEffort = leftFeedforward.calculate(Units.degreesToRadians(leftTarget), 0.0);
        var pidEffort = leftController.calculate(getLeftEncoder(), leftTarget);

        return ffEffort + pidEffort;
    }

 
    private double getRightEncoder(){
        return rightEncoder.getPosition();
    }

    private double getRightEffort(){
        var ffEffort = rightFeedforward.calculate(Units.degreesToRadians(rightTarget), 0.0);
        var pidEffort = rightController.calculate(getRightEncoder(), rightTarget);

        return ffEffort + pidEffort;
    }


    /*Periodic, do we really need it? old code only had troubleshooting stuff in periodic */
    // @Override
    // public void periodic() {    }

    private void configArmSubsys() {
      //right motor config
      rightMotor.setIdleMode(IdleMode.kBrake);
      rightMotor.setInverted(true);
      rightMotor.setSmartCurrentLimit(kMotorCurrentLimit);
      rightMotor.enableVoltageCompensation(rightMotor.getBusVoltage());

      //left motor config
      leftMotor.setIdleMode(IdleMode.kBrake);
      leftMotor.setInverted(true);
      leftMotor.setSmartCurrentLimit(kMotorCurrentLimit);
      leftMotor.enableVoltageCompensation(leftMotor.getBusVoltage());

      //encoder config
      rightEncoder.setPositionConversionFactor(kHeightConvFact);
      leftEncoder.setPositionConversionFactor(kHeightConvFact);

      //controls config
      rightController.setP(kP);
      rightController.setI(kI);
      rightController.setD(kD);
      
      leftController.setP(kP);
      leftController.setI(kI);
      leftController.setD(kD);

      //burning flashes 
      rightMotor.burnFlash();
      leftMotor.burnFlash();

    }
}