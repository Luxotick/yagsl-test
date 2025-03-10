package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    //private final SparkMax commonMotor;
    //private final SparkMax algaeMotor;
    //private final SparkMax coralMotor;
    private final SparkMax rotateMotor;
    private boolean isRunning = false;
    
    // Constants for direct motor control
    private static final double ROTATION_SPEED = 0.3;  // Default rotation speed (0-1)
    private static final double MAX_POSITION = 100.0;   // Maximum allowed position
    private static final double MIN_POSITION = -100.0;    // Minimum allowed position
    
    public ShooterSubsystem() {
        // Initialize motors with their CAN IDs
        //commonMotor = new SparkMax(43, MotorType.kBrushless);
        //algaeMotor = new SparkMax(44, MotorType.kBrushless);
        //coralMotor = new SparkMax(44, MotorType.kBrushless);
        rotateMotor = new SparkMax(41, MotorType.kBrushless);

        // Configure the rotation motor
        SparkMaxConfig rotateConfig = new SparkMaxConfig();
        rotateConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake);  // Set to brake mode for better position holding
        
        rotateMotor.configure(rotateConfig, null, null);
        
        // Reset encoder position to zero
        if (rotateMotor.getEncoder() != null) {
            rotateMotor.getEncoder().setPosition(0);
        } else {
            System.err.println("WARNING: Encoder not found on rotate motor!");
        }
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
        //coralMotor.configure(coralConfig, null, null);

        //commonMotor.configure(commonConfig, null,  null);

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
       //coralMotor.configure(algaeConfig, null, null);

       //commonMotor.configure(commonConfig, null,  null);

      isRunning = true;
    }

    /**
     * Rotate the shooter up (positive direction) at a fixed speed
     */
    public void rotateUp() {
        // Check if we're at the maximum position limit
        if (rotateMotor.getEncoder() != null) {
            double currentPosition = rotateMotor.getEncoder().getPosition();
            if (currentPosition >= MAX_POSITION) {
                stopRotation();
                return;
            }
        }
        
        // Set motor to rotate up at the specified speed
        rotateMotor.set(ROTATION_SPEED);
    }

    /**
     * Rotate the shooter down (negative direction) at a fixed speed
     */
    public void rotateDown() {
        // Check if we're at the minimum position limit
        if (rotateMotor.getEncoder() != null) {
            double currentPosition = rotateMotor.getEncoder().getPosition();
            if (currentPosition <= MIN_POSITION) {
                stopRotation();
                return;
            }
        }
        
        // Set motor to rotate down at the specified speed
        rotateMotor.set(-ROTATION_SPEED);
    }

    /**
     * Stop the rotation motor
     */
    public void stopRotation() {
        rotateMotor.set(0);
    }

    /**
     * Get the current position of the shooter
     * @return The current position
     */
    public double getCurrentPosition() {
        if (rotateMotor.getEncoder() != null) {
            return rotateMotor.getEncoder().getPosition();
        }
        return 0.0;
    }

    /**
     * Reset the shooter position to zero
     */
    public void resetPosition() {
        if (rotateMotor.getEncoder() != null) {
            rotateMotor.getEncoder().setPosition(0);
            System.out.println("Shooter position reset to zero");
        }
    }

    public void stopMotors() {
        //commonMotor.set(0);
        //algaeMotor.set(0);
        //coralMotor.set(0);
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
        
        // Safety check for position limits
        if (rotateMotor.getEncoder() != null) {
            double currentPosition = rotateMotor.getEncoder().getPosition();
            
            // If we're moving and hit a limit, stop the motor
            if (currentPosition >= MAX_POSITION && rotateMotor.get() > 0) {
                rotateMotor.set(0);
                System.out.println("Reached maximum position limit");
            } else if (currentPosition <= MIN_POSITION && rotateMotor.get() < 0) {
                rotateMotor.set(0);
                System.out.println("Reached minimum position limit");
            }
        }
    }
} 