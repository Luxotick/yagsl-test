package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor1;
    private final SparkMax elevatorMotor2;
    private final RelativeEncoder encoder1;
    private final SparkClosedLoopController pidController1;
    
    // Define elevator positions in degrees
    public static final double POSITION_1 = 0.0;    // Home position
    public static final double POSITION_2 = 870.0;   // Mid-low position
    public static final double POSITION_3 = 90.0;   // Mid-high position
    public static final double POSITION_4 = 135.0;  // Maximum height
    
    // PID coefficients
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    
    // Output range
    private static final double kMinOutput = -0.5;
    private static final double kMaxOutput = 0.5;
    
    private double targetPosition = POSITION_1;

    public ElevatorSubsystem() {
        // Initialize motors with their CAN IDs
        elevatorMotor1 = new SparkMax(61, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(62, MotorType.kBrushless);
        
        // Create configuration
        SparkMaxConfig config1 = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        // Configure motors
        config1
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
            
        // Configure motor2 to follow motor1 inverted
        config2
            .follow(61, true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

            
        // Configure encoder
        config1.encoder
            .positionConversionFactor(360.0 / 25.0); // Converts to degrees
            
        // Configure PID
        config1.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .iZone(kIz)
            .outputRange(kMinOutput, kMaxOutput);
            
        // Apply configuration
        elevatorMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Get encoder and PID controller references
        encoder1 = elevatorMotor1.getEncoder();
        pidController1 = elevatorMotor1.getClosedLoopController();
    }
    
    public void setPosition(double targetDegrees) {
        targetPosition = targetDegrees;
        pidController1.setReference(targetDegrees, ControlType.kPosition);
    }
    
    public void setToPosition1() {
        setPosition(POSITION_1);
    }
    
    public void setToPosition2() {
        setPosition(POSITION_2);
    }
    
    public void setToPosition3() {
        setPosition(POSITION_3);
    }
    
    public void setToPosition4() {
        setPosition(POSITION_4);
    }
    
    public double getCurrentPosition() {
        return encoder1.getPosition();
    }
    
    public boolean isAtSetpoint() {
        return Math.abs(getCurrentPosition() - targetPosition) < 2.0; // 2 degree tolerance
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public void resetEncoder() {
        encoder1.setPosition(0);
    }
}
