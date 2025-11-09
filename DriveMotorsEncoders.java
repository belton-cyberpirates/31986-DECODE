package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Collections;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.PIDController;


public class DriveMotors {

	enum states {
		ENCODERS,
		DISTANCE,
		POWER,
		IDLE
	}

	public static PIDController distanceSensorPidController = new PIDController(0, 0, 0);
	public static PIDController forwardPidController = new PIDController(0.00255, 0.00000033, 0.0000025);
	public static PIDController strafePidController = new PIDController(0.00265, 0.0000004, 0.000004);
	public static PIDController imuPidController = new PIDController(1.2, 0, 0.001);

	static Orientation angles;

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	public GoBildaPinpointDriver odometry;

	private LinearOpMode auto;

	private DistanceSensor distSensor;

	ElapsedTime deltaTimer = new ElapsedTime();

	public states state = states.IDLE;

	public double targetX;
	public double targetY;
	public double targetHeading;

	public int targetDistance;
	
	ElapsedTime odometryTimer = new ElapsedTime();
	
	public double heading = 0;


	public DriveMotors(LinearOpMode auto) {
		this.auto = auto;

		this.frontRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_RIGHT_WHEEL_NAME);
		this.frontLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_LEFT_WHEEL_NAME);
		this.backLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_LEFT_WHEEL_NAME);
		this.backRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_RIGHT_WHEEL_NAME);
		
		this.backLeft.setTargetPosition(0);
		this.frontLeft.setTargetPosition(0);
		this.frontRight.setTargetPosition(0);
		this.backRight.setTargetPosition(0);
	
		ResetEncoders();
		SetZeroBehaviour();
	}


	public double process() {
		double deltaTime = deltaTimer.seconds();

		switch (this.state) {
			case DISTANCE:
				SetToRunWithPower();
				driveWithDistanceSensor(deltaTime);
				break;
		}
		
		this.odometry.update();
		deltaTimer.reset();
		
		return deltaTime;
	}


	public void DriveWithPower(double backLeftPower, double frontLeftPower, double frontRightPower, double backRightPower) {
		this.state = states.POWER;

		backLeft.setPower(backLeftPower);
		frontLeft.setPower(frontLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}


	public void MoveWithEncoders(int backLeftPos, int frontLeftPos, int frontRightPos, int backRightPos) {
		backLeft.setTargetPosition(backLeft.getTargetPosition() + backLeftPos);
		frontLeft.setTargetPosition(frontLeft.getTargetPosition() + frontLeftPos);
		frontRight.setTargetPosition(frontRight.getTargetPosition() + frontRightPos);
		backRight.setTargetPosition(backRight.getTargetPosition() + backRightPos);
	}


	public void MoveWithEncoders(Direction dir, double distance) {
		switch (dir) {
			case FORWARD:
				MoveWithEncoders(-distance, -distance, distance, distance);
				break;
 			case LEFT:
				MoveWithEncoders(distance, -distance, -distance, distance);
				break;
            case RIGHT:
				MoveWithEncoders(-distance, distance, distance, -distance);
				break;
 			case BACKWARD:
				MoveWithEncoders(distance, distance, -distance, -distance);
				break;
			case FRONT_LEFT:
				MoveWithEncoders(distance, 0, -distance, 0);
				break;
  			case FRONT_RIGHT:
				MoveWithEncoders(0, distance, 0, -distance);
				break;
  			case BACK_LEFT:
				MoveWithEncoders(0, -distance, 0, distance);
				break;
 			case BACK_RIGHT:
				MoveWithEncoders(-distance, 0, distance, 0);
				break;
		}
	}


	public void driveWithDistanceSensor(double delta) {
		double dist = distSensor.getDistance(DistanceUnit.MM);
		double error = targetDistance - dist;
		
		double power = distanceSensorPidController.PIDControl(error, delta);

		frontLeft.setPower(-power);
		frontRight.setPower(power);
		backLeft.setPower(-power);
		backRight.setPower(power);
		
		auto.telemetry.addData("dist", dist);
		auto.telemetry.addData("error", error);
		auto.telemetry.addData("power", power);
	}


	public void Move(double xPos, double yPos, double heading) {
		this.targetX = xPos;
		this.targetY = yPos;
		this.targetHeading = -( heading * ( Math.PI / 180 ) );

		this.odometryTimer.reset();

		this.state = states.ODOMETRY;
	}


	public void MoveToDistance(int distance) {
		this.targetDistance = distance;

		this.state = states.DISTANCE;
	}


	public void Stop() {
		this.state = states.IDLE;
	}


	public boolean isDone() {
		switch (this.state) {
			case ENCODERS:
				return !backLeft.isBusy() && 
					!frontLeft.isBusy() && 
					!frontRight.isBusy() && 
					!backRight.isBusy(); 
			
			case DISTANCE:
				return (Math.abs(distanceSensorPidController.lastError) < 5);
		}
		
		return true;
	}


	private void SetToRunWithPower() {
		this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}


	private void SetToRunWithVelocity() {
		this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
  
  
	private void SetZeroBehaviour() {
		this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}
	
	
	private void ResetEncoders() {
		this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}
}
