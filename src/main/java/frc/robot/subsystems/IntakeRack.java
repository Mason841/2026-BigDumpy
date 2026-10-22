// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorLogUtil;
import frc.robot.constants.SuperstructureConstants;

public class IntakeRack extends SubsystemBase {
  
	private TalonFX leftMotorLeader = new TalonFX(SuperstructureConstants.IDs.INTAKERACK_LEFT_MOTOR_ID);
	private TalonFX rightMotor = new TalonFX(SuperstructureConstants.IDs.INTAKERACK_RIGHT_MOTOR_ID);

	private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
	private MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);
	// private MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
	// private PositionVoltage positionControl = new PositionVoltage(0);
	private VoltageOut voltageControl = new VoltageOut(0);

	public enum IntakeRackState {
		STOP,
		IDLE,
		EXTENDED_FULL,
		EXTENDED_HALF,
		STOWED_MATCHSTART,
		STOWED_FIRING,
		FOLLOW_OVERRIDE,
		FORCE_IN_SLOW,
		FORCE_IN_FAST,
		FORCE_OUT_SLOW,
		FORCE_OUT_FAST,
		PUSH_IN_SLOW
	}

	private enum IntakeRackControlMode {
		IDLE,
		POSITION,
		VELOCITY,
		VOLTAGE
	}

	private IntakeRackState state = IntakeRackState.STOWED_FIRING;
	private IntakeRackControlMode controlMode = IntakeRackControlMode.POSITION;

	private double overridePosition = 0;

	private double targetVelocity = 0;
	private double targetPosition = 0;
	private double targetVoltage = 0;

	public IntakeRack() {
		leftMotorLeader.getConfigurator().apply(SuperstructureConstants.IntakeRackConstants.INTAKERACK_MOTOR_CONFIGS);
		rightMotor.getConfigurator().apply(SuperstructureConstants.IntakeRackConstants.INTAKERACK_MOTOR_CONFIGS_INVERTED);

		this.zeroMotors();
	}

	public void setState(IntakeRackState wantedState) {
		state = wantedState;
	}

	public void setOverridePosition(double wantedPosition) {
		overridePosition = wantedPosition;
	}

	public IntakeRackState getState() {
		return state;
	}

	public double getPosition() {
		return this.leftMotorLeader.getPosition().getValueAsDouble();
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public void stopMotors() {
		this.targetVelocity = 0;
		this.leftMotorLeader.stopMotor();
		this.rightMotor.stopMotor();
	}

	public void zeroMotors() {
		leftMotorLeader.setPosition(0);
		rightMotor.setPosition(0);
	}

	public StatusCode[] setControl(ControlRequest control) {
		return new StatusCode[] {
			this.leftMotorLeader.setControl(control),
			this.rightMotor.setControl(control)
		};
	}

	public void periodicRun() {
		Logger.recordOutput("IntakeRack/LeftMotorPosition", this.leftMotorLeader.getPosition().getValueAsDouble());
		Logger.recordOutput("IntakeRack/RightMotorPosition", this.rightMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("IntakeRack/TargetPosition", this.getTargetPosition());
		Logger.recordOutput("IntakeRack/State", this.getState());
		MotorLogUtil.logMotor(leftMotorLeader, "IntakeRack", "LeftIntakeRackMotor");
		MotorLogUtil.logMotor(rightMotor, "IntakeRack", "RightIntakeRackMotor");

		switch (state) {
			case STOP:
				controlMode = IntakeRackControlMode.VELOCITY;
				targetVelocity = 0;
				break;
		
			case IDLE:
				controlMode = IntakeRackControlMode.IDLE;
				targetVelocity = 0;
				break;
				
			case EXTENDED_FULL:
				controlMode = IntakeRackControlMode.POSITION;
				targetPosition = SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX;
				break;

			case EXTENDED_HALF:
				controlMode = IntakeRackControlMode.POSITION;
				targetPosition = SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX / 2;
				break;

			case STOWED_MATCHSTART:
				controlMode = IntakeRackControlMode.POSITION;
				targetPosition = SuperstructureConstants.IntakeRackConstants.INTAKERACK_MIN;
				break;

			case STOWED_FIRING:
				controlMode = IntakeRackControlMode.POSITION;
				targetPosition = SuperstructureConstants.IntakeRackConstants.INTAKERACK_MIN + 
					(SuperstructureConstants.IntakeRackConstants.INTAKERACK_MAX * 0.3);
				break;

			case FOLLOW_OVERRIDE:
				controlMode = IntakeRackControlMode.POSITION;
				targetPosition = overridePosition;
				break;

			case FORCE_IN_SLOW:
				controlMode = IntakeRackControlMode.VELOCITY;
				targetVelocity = -20;
				break;

			case FORCE_IN_FAST:
				controlMode = IntakeRackControlMode.VELOCITY;
				targetVelocity = -70;
				break;

			case FORCE_OUT_SLOW:
				controlMode = IntakeRackControlMode.VELOCITY;
				targetVelocity = 20;
				break;

			case FORCE_OUT_FAST:
				controlMode = IntakeRackControlMode.VELOCITY;
				targetVelocity = 70;
				break;

			case PUSH_IN_SLOW:
				controlMode = IntakeRackControlMode.VOLTAGE;
				targetVoltage = -1;
				break;

			default:
				break;
		}

		switch (controlMode) {
			case IDLE:
				this.stopMotors();
				break;
			
			case POSITION:
				this.leftMotorLeader.setControl(positionControl.withPosition(targetPosition));
				this.rightMotor.setControl(positionControl.withPosition(targetPosition));
				break;

			case VELOCITY:
				this.leftMotorLeader.setControl(velocityControl.withVelocity(targetVelocity));
				this.rightMotor.setControl(velocityControl.withVelocity(targetVelocity));
				break;

			case VOLTAGE:
				this.leftMotorLeader.setControl(voltageControl.withOutput(targetVoltage));
				this.rightMotor.setControl(voltageControl.withOutput(targetVoltage));				
				break;
		
			default:
				this.stopMotors();
				break;
		}
	}
}
