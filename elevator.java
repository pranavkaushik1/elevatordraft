// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Elevator extends SubsystemBase {


  SparkMax elevatorUpMotor = new SparkMax(Constants.kElevatorUp, MotorType.kBrushless);
  SparkMax elevatorDownMotor = new SparkMax(Constants.kElevatorDown, MotorType.kBrushless);
  SparkMaxConfig elevatorUpConfig;
  SparkMaxConfig elevatorDownConfig;
  RelativeEncoder e_UpEncoder = elevatorUpMotor.getEncoder();
  RelativeEncoder e_DownEncoder = elevatorDownMotor.getEncoder();

  private double elevator_kP = Constants.ELEVATOR_PID_CONSTANTS[0];
  private double elevator_kI = Constants.ELEVATOR_PID_CONSTANTS[1];
  private double elevator_kD = Constants.ELEVATOR_PID_CONSTANTS[2];

  private double elevator_kS = Constants.ELEVATOR_FEED_FORWARD_CONSTANTS[0];
  private double elevator_kG = Constants.ELEVATOR_FEED_FORWARD_CONSTANTS[1];
  private double elevator_kV = Constants.ELEVATOR_FEED_FORWARD_CONSTANTS[2];
  private double elevator_kA = Constants.ELEVATOR_FEED_FORWARD_CONSTANTS[3];

  ProfiledPIDController e_PidController = new ProfiledPIDController(elevator_kP, elevator_kI, elevator_kD, new Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration));

  ElevatorFeedforward e_Feedforward = new ElevatorFeedforward(elevator_kS, elevator_kG, elevator_kV, elevator_kA);

  // set min and max elevator heights
  // set up sysid routine and setup (we need this?)


  public Elevator() {
    elevatorUpConfig = new SparkMaxConfig();
    elevatorDownConfig = new SparkMaxConfig();

    elevatorUpConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);
    
    elevatorDownConfig
      .apply(elevatorUpConfig)
      .inverted(true);

    // elevatorUpConfig.closedLoop
    //   .pid(elevator_kP, elevator_kI, elevator_kD)
    //   .outputRange(-0.75, 0.75)
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    // elevatorDownConfig.closedLoop
    //   .pid(elevator_kP, elevator_kI, elevator_kD)
    //   .outputRange(-0.75, 0.75)
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    elevatorUpMotor.configure(elevatorUpConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorDownMotor.configure(elevatorDownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMid(double position) {
    // getting kp value, not sure if this value will work great, we have to test and check
    //no way to know until we test it

    //do 1(max output)/(1/3 * max height)
    //so we are at max output until error is less than 33% of the max height, then we start slowing
    double kp = 3/Constants.MAX_ELEVATOR_HEIGHT;

      double destination = Constants.MID_POSITION;

    // these two js make sure that the elevator position is within the bounds
    // we need to ask malla if the min and max will be the absolute min and max of the elevator
    // if thats the case then theres obv no need for these lines because it'll
    // always be in the bounds
    if (position > Constants.MAX_ELEVATOR_HEIGHT) {
        position = Constants.MAX_ELEVATOR_HEIGHT;
    }

    if (position < Constants.MAX_ELEVATOR_HEIGHT) {
        position = Constants.MAX_ELEVATOR_HEIGHT;
    }

    double currentPosition = e_UpEncoder.getPosition(); // we use up encoder for getting position im pretty sure
    //double velocity = e_PidController.calculate(currentPosition, position);
    //double acceleration = e_PidController.getSetpoint().velocity;
    double error = destination - currentPosition;
    //double feedForward = e_Feedforward.calculate(velocity, acceleration);  // ok so when i looked online, feedForward takes in
    // both velocity and acceleration.

    // this is the code for accel i found online but because
    // i dont rlly get how it works, ill js keep it here:
    // double acceleration = e_PidController.getSetpoint().velocity;





    double motorOutput = error*(kp); // this will set our motor speed based on the PID and feedForward shit
    if(motorOutput > 1.00)
      motorOutput = 1.00;
    else if(motorOutput < -1.00)
      motorOutput = -1.00;

    if(motorOutput>0) {
      elevatorUpMotor.set(motorOutput);
      elevatorDownMotor.set(0);
    }
    else if(motorOutput<0){
      elevatorDownMotor.set(-motorOutput); 
      elevatorUpMotor.set(0);
    }
      
      // inverted motor

    // im missing tolerance because if the distance to the target position is too less it could move and stop abruptly
    // we can implement this later

}

public void reset(double currentPosition) {

  DigitalInput limitSwitch = new DigitalInput(0);

  if (limitSwitch.get()) {
    currentPosition = 0;
    motorOutput.set(0);
  }
}

public void setSP(double position) {
  // getting kp value, not sure if this value will work great, we have to test and check
  //no way to know until we test it

  //do 1(max output)/(1/3 * max height)
  //so we are at max output until error is less than 33% of the max height, then we start slowing
  double kp = 3/Constants.MAX_ELEVATOR_HEIGHT;

    double destination = Constants.SCORING_POSITION;

  // these two js make sure that the elevator position is within the bounds
  // we need to ask malla if the min and max will be the absolute min and max of the elevator
  // if thats the case then theres obv no need for these lines because it'll
  // always be in the bounds
  if (position > Constants.MAX_ELEVATOR_HEIGHT) {
      position = Constants.MAX_ELEVATOR_HEIGHT;
  }

  if (position < Constants.MAX_ELEVATOR_HEIGHT) {
      position = Constants.MAX_ELEVATOR_HEIGHT;
  }

  double currentPosition = e_UpEncoder.getPosition(); // we use up encoder for getting position im pretty sure
  //double velocity = e_PidController.calculate(currentPosition, position);
  //double acceleration = e_PidController.getSetpoint().velocity;
  double error = destination - currentPosition;
  //double feedForward = e_Feedforward.calculate(velocity, acceleration);  // ok so when i looked online, feedForward takes in
  // both velocity and acceleration.

  // this is the code for accel i found online but because
  // i dont rlly get how it works, ill js keep it here:
  // double acceleration = e_PidController.getSetpoint().velocity;

  double motorOutput = error*(kp); // this will set our motor speed based on the PID and feedForward shit
  if(motorOutput > 1.00)
    motorOutput = 1.00;
  else if(motorOutput < -1.00)
    motorOutput = -1.00;

  if(motorOutput>0) {
    elevatorUpMotor.set(motorOutput);
    elevatorDownMotor.set(0);
  }
  else if(motorOutput<0){
    elevatorDownMotor.set(-motorOutput); 
    elevatorUpMotor.set(0);
  }
    
    // inverted motor

  // im missing tolerance because if the distance to the target position is too less it could move and stop abruptly
  // we can implement this later

}




  public void stop() {
    elevatorUpMotor.set(0);
    elevatorDownMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
