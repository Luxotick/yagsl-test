package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubystem extends SubsystemBase {
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;
    private boolean isRunning = false;
    private boolean speed = false;

    public HookSubystem() {
        // Initialize motors with their CAN IDs
        motor1 = new SparkMax(31, MotorType.kBrushless);
        motor2 = new SparkMax(32, MotorType.kBrushless);
        encoder = motor1.getEncoder();
        
    }

    public void startMotors() {
        SparkMaxConfig motor2Config = new SparkMaxConfig();
         motor2Config
        .smartCurrentLimit(60)
        .inverted(true);    
        SparkMaxConfig motor1Config = new SparkMaxConfig();
        motor1Config
        .smartCurrentLimit(60)
        .inverted(false);   
        // Reset motor controllers to factory defaults
        SmartDashboard.putNumber("sa", encoder.getPosition());


        // Set motor2 to run in opposite direction
        motor2.configure(motor2Config, null, null);

        motor1.configure(motor1Config, null,  null);
        if(speed){
            motor1.set(0.6); // Run at 50% speed - adjust as needed
            motor2.set(0.6); // This wil++l run in opposite direction due to inversion    
        }else{
            motor1.set(0.03); // Run at 50% speed - adjust as needed
            motor2.set(0.03); // This will run in opposite direction due to inversion    
        }
       isRunning = true;
    }

    public void startMotorInverted() {
        SparkMaxConfig motor2InvertedConfig = new SparkMaxConfig();
        motor2InvertedConfig
       .smartCurrentLimit(40)
       .inverted(false);    
       SparkMaxConfig motor1InvertedConfig = new SparkMaxConfig();
       motor1InvertedConfig
       .smartCurrentLimit(40)
       .inverted(true);   

        motor1.configure(motor1InvertedConfig, null, null);
        motor2.configure(motor2InvertedConfig, null, null);
        motor1.set(0.03); // Run at 50% speed - adjust as needed
        motor2.set(0.03); // This will run in opposite direction due to inversion
        isRunning = true;
    }

    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
        isRunning = false;
    }

    public void toggleMotors() {
        if (isRunning) {
            stopMotors();
        } else {
            startMotors();
        }
    }

    public void toggleMotorsReverse() {
        if (isRunning) {
            stopMotors();
        } else {
            startMotorInverted();
        }
    }

    public void changeSpeed() {
        speed = !speed;
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
} 