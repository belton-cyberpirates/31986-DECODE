package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BotConfig;
import java.util.List;

public class Intake {

	private LinearOpMode auto;

	public DcMotorEx intake;
	public DcMotorEx pusher;
	public Servo stopper;


	public Intake(LinearOpMode auto) {
		this.auto = auto;
		
		this.intake = auto.hardwareMap.get(DcMotorEx.class, BotConfig.INTAKE_NAME);
		this.pusher = auto.hardwareMap.get(DcMotorEx.class, BotConfig.PUSHER_NAME);
		this.stopper = auto.hardwareMap.get(Servo.class, BotConfig.STOPPER_NAME);
	}
  
  
	public void SetIntakeVelocity(int velocity) {
		intake.setVelocity(velocity);
	}


	public void SpinIntake() {
		this.SetIntakeVelocity(BotConfig.PUSHER_VELOCITY)
	}
  
  
	public void SetPusherVelocity(int velocity) {
		pusher.setVelocity(velocity);
	}


	public void SpinPusher() {
		this.SetPusherVelocity(BotConfig.INTAKE_VELOCITY)
	}


	public void OpenStopper() {
		stopper.setPosition(BotConfig.STOPPER_OPEN_POS);
	}


	public void CloseStopper() {
		stopper.setPosition(BotConfig.STOPPER_CLOSE_POS);
	}


	public boolean isStopperOpen() {
		return stopper.getPosition() == BotConfig.STOPPER_OPEN_POS;
	}
	
	public int getIntakeVelocity() {
		return wrist.getCurrentPosition();
	}
}