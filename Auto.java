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
	public Intake intake;
	public Launcher launcher;
	
	/**
	 * Initialize classes used by autos
	 */
	protected void Initialize() {
		// Create robot component classes
		// Add, remove, modify depending on how your robot works
		driveMotors = new DriveMotors(this);
		intake = new Intake(this);
		launcher = new Launcher(this);

		// Run class initialization funcs
		driveMotors.InitializeOdometry();

		// Let the user know when initialization is done
		telemetry.addData("Beginning Initialization...", "");
		telemetry.addData("Ready to run!", "");
		telemetry.update();
	}

	@Override
	public void runOpMode() {
		Initialize();

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

			// Process classes
			// Add, remove, modify depending on how your robot works
			driveMotors.process();
			
			// Loop telemetry; updates constantly
			telemetry.addData("X pos", driveMotors.odometry.getPosX());
			telemetry.addData("Y pos", driveMotors.odometry.getPosY());
			telemetry.addData("Launcher Velocity", launcher.getVelocity());

			telemetry.update();
		}
	}
}
