package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  XboxController driver = new XboxController(0);
  TalonFX arm = new TalonFX(0);
  TalonFX rightFront = new TalonFX(4);
  TalonFX rightBack = new TalonFX(3);
  TalonFX leftFront = new TalonFX(2);
  TalonFX leftBack = new TalonFX(1);
  DifferentialDrive tank = new DifferentialDrive(leftFront, rightFront);
  double driveControllerDeadband = 0.04;
  double minJoystickDriveResponse = 0.24;
  double maxTurnPower = 0.6;
  double armPos = 0.0;

  public void robotInit() {
    configMotor(rightFront, true);
    configMotor(rightBack, true);
    configMotor(leftFront, false);
    configMotor(leftBack, false);
    configMotor(arm, false);
    leftBack.setControl(new Follower(2, false));
    rightBack.setControl(new Follower(4, false));
    armPos = arm.getRotorPosition().getValueAsDouble();
  }

  public void robotPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void teleopInit() {}

  public void teleopPeriodic() {
    double driveInput = -driver.getLeftY();
    double turnInput = -driver.getRightX();
    double driveOutput = 0.0;
    if (Math.abs(driveInput) > driveControllerDeadband) {
      if (driveInput > 0) {
        driveOutput = minJoystickDriveResponse+(1.0-minJoystickDriveResponse)*Math.pow(driveInput, 2);
      } else {
        driveOutput = -minJoystickDriveResponse-(1.0-minJoystickDriveResponse)*Math.pow(driveInput, 2);
      }
   }
   double turnOutput = 0.0;
   if (Math.abs(turnInput) > driveControllerDeadband) {
     if (turnInput > 0) {
       turnOutput = minJoystickDriveResponse+(maxTurnPower-minJoystickDriveResponse)*Math.pow(turnInput, 2);
     } else {
      turnOutput = -minJoystickDriveResponse-(maxTurnPower-minJoystickDriveResponse)*Math.pow(turnInput, 2);
     }
   }
    tank.arcadeDrive(driveOutput, turnOutput);
    arm.setControl(new MotionMagicDutyCycle(armPos));
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void configMotor(TalonFX motor, boolean isInverted) {
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfigs.Slot0.kP = 0.8;
    motorConfigs.Slot0.kI = 2.0;
    motorConfigs.Slot0.kD = 0.006;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 200.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 500.0;
    motorConfigurator.apply(motorConfigs);
  }
}