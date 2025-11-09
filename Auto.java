package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Heading;


public abstract class Auto extends LinearOpMode {
	abstract Action[] getActions();

	public DriveMotors driveMotors;
	public Arm arm;
	public Intake intake;
	public Heading heading;
	
	/**
	 * Initialize classes used by autos
	 */
	protected void Initialize() {
		driveMotors = new DriveMotors(this);
		arm = new Arm(this);
		intake = new Intake(this);

		driveMotors.InitializeOdometry();

		telemetry.addData("Beginning Initialization...", "");
		telemetry.addData("DO NOT START AUTONOMOUS YET!", "");
		telemetry.update();
	}

	/**
	 * Set reliable initial configuration for robot motors
	 */
	protected void MotorSetup() {
        

		telemetry.addData("Fully Initialized", "");
		telemetry.update();
	}

	protected void saveHeading() {
		double _heading = driveMotors.odometry.getHeading();
		this.heading.setHeading(_heading);
	}

	@Override
	public void runOpMode() {
		Initialize();
		MotorSetup();

		waitForStart();

		Action[] actions = getActions();
		Action currentAction = null;

		while (opModeIsActive() && ( actions.length > 0 )) { // <----------------------------------------------------------------
			if (currentAction == null) {
				currentAction = actions[0];
				currentAction.onStart();
			}
			else {
				currentAction.process();
			}
			
			if ( actions[0].isDone() ) {
				currentAction = null;
				actions = Arrays.copyOfRange(actions, 1, actions.length);
			}

			driveMotors.process();
			arm.process();
			
			telemetry.addData("X pos", driveMotors.odometry.getPosX());
			telemetry.addData("Y pos", driveMotors.odometry.getPosY());
			telemetry.addData("Arm Height", arm.getHeight());
			telemetry.addData("Wrist Pos", intake.getWristPos());

			telemetry.update();
		}

		saveHeading();
	}
}