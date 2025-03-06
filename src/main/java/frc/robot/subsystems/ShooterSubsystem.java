package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax commonMotor;
    private final SparkMax algaeMotor;
    private final SparkMax coralMotor;
    private boolean isRunning = false;

    public ShooterSubsystem() {
        // Initialize motors with their CAN IDs
        commonMotor = new SparkMax(43, MotorType.kBrushless);
        algaeMotor = new SparkMax(44, MotorType.kBrushless);
        coralMotor = new SparkMax(44, MotorType.kBrushless);

        
    }

    public void startCoralMotors() {
        SparkMaxConfig commonConfig = new SparkMaxConfig();
         commonConfig
        .smartCurrentLimit(40)
        .inverted(false);    
        SparkMaxConfig coralConfig = new SparkMaxConfig();
        coralConfig
        .smartCurrentLimit(40)
        .follow(0, true);
        // Reset motor controllers to factory defaults


        // Set motor2 to run in opposite direction
        coralMotor.configure(coralConfig, null, null);

        commonMotor.configure(commonConfig, null,  null);

       isRunning = true;
    }

    public void startAlgaeMotors(){
        SparkMaxConfig commonConfig = new SparkMaxConfig();
        commonConfig
       .smartCurrentLimit(40)
       .inverted(false);    
       SparkMaxConfig algaeConfig = new SparkMaxConfig();
       algaeConfig
       .smartCurrentLimit(40)
       .follow(0, true);
       // Reset motor controllers to factory defaults


       // Set motor2 to run in opposite direction
       coralMotor.configure(algaeConfig, null, null);

       commonMotor.configure(commonConfig, null,  null);

      isRunning = true;
    }

    public void stopMotors() {
        commonMotor.set(0);
        algaeMotor.set(0);
        coralMotor.set(0);
        isRunning = false;
    }

    public void toggleCoralMotors() {
        if (isRunning) {
            stopMotors();
        } else {
            startCoralMotors();
        }
    }

    public void toggleAlgaeMotors()
    {
        if (isRunning) {
            stopMotors();
        } else {
            startAlgaeMotors();
        }
    }
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
} 