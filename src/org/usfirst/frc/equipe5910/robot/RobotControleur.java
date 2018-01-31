package org.usfirst.frc.equipe5910.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotControleur extends IterativeRobot {
	
	public static final double DISTANCE_KP = 0.18; //0.11;
	public static final double DISTANCE_KI = 0.00045; //0.00045;
	public static final float DISTANCE_TOLERANCE = 0.083f;

	public class SortiePID implements PIDOutput {

		double distanceSortiePID;
		
		@Override
		public void pidWrite(double sortie) {
			distanceSortiePID = sortie;
		}
		
		public double getPIDOut() {
			return distanceSortiePID;
		}
	}		
	
	public static final double GYRO_KP = 0.02; //0.03
	public static final double GYRO_KI = 0.0;
	
	public static final double GYRO_KP_ROTATION = 0.00075; // + haut = plus aggressif
	public static final double GYRO_KI_ROTATION = 0.000085; // + bas = plus aggressif
	public static final boolean GYRO_INVERSE = true;
	
	public static final int BOUTON_GYRO_RESET = 1;
	
	// PWM Outputs
	public static final int ROUE_AVANT_GAUCHE = 10; // SP 1
	public static final int ROUE_ARRIERE_GAUCHE = 14; // SP 2
	public static final int ROUE_AVANT_DROIT= 13; // SP 3
	public static final int ROUE_ARRIERE_DROIT = 17; // SP 4
	
	public static final boolean INVERSION_ROUE_AVANT_GAUCHE = true;
	public static final boolean INVERSION_ROUE_ARRIERE_GAUCHE = true;
	public static final boolean INVERSION_ROUE_AVANT_DROIT = false;
	public static final boolean INVERSION_ROUE_ARRIERE_DROIT= false;	
	
	GyroADXRS450 gyro;
	PIDController pidGyro;
	SortiePID gyroSortiePID;
	
    VictorSP roueAvantGauche; 
	VictorSP roueArriereGauche; 
	VictorSP roueAvantDroite; 
	VictorSP roueArriereDroite; 
	
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 * 
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit()");
		 
		roueAvantGauche = new VictorSP(ROUE_AVANT_GAUCHE);
		roueArriereGauche = new VictorSP(ROUE_ARRIERE_GAUCHE);
		roueAvantDroite = new VictorSP(ROUE_AVANT_DROIT);
		roueArriereDroite = new VictorSP(ROUE_ARRIERE_DROIT);
		 	
		roueAvantGauche.setInverted(INVERSION_ROUE_AVANT_GAUCHE); // TRUE
		roueArriereGauche.setInverted(INVERSION_ROUE_ARRIERE_GAUCHE); // TRUE
		roueAvantDroite.setInverted(INVERSION_ROUE_AVANT_DROIT);
		roueArriereDroite.setInverted(INVERSION_ROUE_ARRIERE_DROIT);
		
		 gyro = new GyroADXRS450();
		 gyro.setPIDSourceType(PIDSourceType.kDisplacement);
		 gyroSortiePID = new SortiePID();
		 
		 pidGyro = new PIDController(-GYRO_KP, GYRO_KI_ROTATION, 0, gyro, gyroSortiePID);
		 pidGyro.setSetpoint(0.0f);
		 pidGyro.setAbsoluteTolerance(3);
		 pidGyro.enable();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		System.out.println("autonomousInit()");
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		System.out.println("autonomousPeriodic()");
		// tourner a l'envers
		roueAvantGauche.set(-gyroSortiePID.getPIDOut());
		roueAvantDroite.set(gyroSortiePID.getPIDOut());
	    roueArriereGauche.set(-gyroSortiePID.getPIDOut());
		roueArriereDroite.set(gyroSortiePID.getPIDOut());
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		System.out.println("teleopInit()");
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		System.out.println("teleopPeriodic()");
		// tourner a l'endroit
		roueAvantGauche.set(gyroSortiePID.getPIDOut());
		roueAvantDroite.set(-gyroSortiePID.getPIDOut());
	    roueArriereGauche.set(gyroSortiePID.getPIDOut());
		roueArriereDroite.set(-gyroSortiePID.getPIDOut());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		System.out.println("testPeriodic()");
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testInit() {
		System.out.println("testInit()");
	}
	
}
