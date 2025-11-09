package org.firstinspires.ftc.teamcode;


public class BotConfig {
  /*****************************************************************************
  ** HARDWARE CONSTANTS
  *****************************************************************************/
  public static final String FRONT_RIGHT_WHEEL_NAME = "FR";
  public static final String FRONT_LEFT_WHEEL_NAME = "FL";
  public static final String BACK_LEFT_WHEEL_NAME = "BL";
  public static final String BACK_RIGHT_WHEEL_NAME = "BR";
  
  public static final String FLYWHEEL_LEFT_NAME = "FlyL";
  public static final String FLYWHEEL_RIGHT_NAME = "FlyR";
  public static final String BANANA_NAME = "Banana";
  public static final String INTAKE_NAME = "Intake";

  public static final String IMU_NAME = "imu";
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TILE_LENGTH = 595; // MM
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** SPEED CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double STRAFE_MULT = 1.41;
  public static final int MAX_DRIVE_VELOCITY = 2000;

  public static final int INTAKE_VELOCITY = 4500;
  public static final int LAUNCHER_VELOCITY = 2000;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** BASE CONSTANTS
  *****************************************************************************/
  //public static final int PICKUP_X = 650;
  //public static final int PICKUP_Y = -850;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** INTAKE CONSTANTS
  *****************************************************************************/
  public static final double BANANA_OPEN_POS = 0.3;
  public static final double BANANA_CLOSE_POS = 0.6;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ODOMETRY CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double FORWARD_OFFSET = 16.25;
  public static final double TRACK_WIDTH = 367;
  public static final double WHEEL_DIAMETER = 38;
  public static final double TICKS_PER_REVOLUTION = 2048;
  public static final double TICKS_PER_MM = (int)( TICKS_PER_REVOLUTION / ( Math.PI * WHEEL_DIAMETER ) );
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CONSTANTS
  *****************************************************************************/
  public static final String CAMERA_NAME = "Webcam 1";
	public static final int CAMERA_RESO_X = 640;
	public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/teampiece.tflite";
	public static final String[] LABELS = { // Define the labels used in our model (must be in training order!)
		"BLUE",
		"RED",
	};
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CALIBRATION CONSTANTS
  * Lens intrinsics
  * UNITS ARE PIXELS
  * NOTE: this calibration is for the C920 webcam at 800x448.
  * You will need to do your own calibration for other configurations!

  Resolution: 1280x720
  Pixel Size: 2.8um
  Sensor Size: 3.58x2.02mm
  Stock lens focal length: 4.2mm
  *****************************************************************************/
  public static final double FX = 1430;
  public static final double FY = 1430;
  public static final double CX = 480;
  public static final double CY = 620;

  public static final double TAGSIZE = 0.166;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DETECTION CONSTANTS
  *****************************************************************************/
  public static final float DECIMATION_HIGH = 3;
  public static final float DECIMATION_LOW = 2;
  public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
  public static final int NUM_FRAMES_BEFORE_LOW_DECIMATION = 4;
  public static final int MAX_NUM_FRAMES_NO_DETECTION = 100; // How many attempts to detect before giving up
  // ---------------------------------------------------------------------------
}