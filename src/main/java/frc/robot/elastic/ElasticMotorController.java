package frc.robot.elastic;
import frc.utils.configfile.Option;
import frc.utils.configfile.OptionsController;
import frc.utils.configfile.OptionsRegistry;
public class ElasticMotorController extends OptionsController {
  public final Option<Boolean> homeButton;
  public final Option<Boolean> goToIdle;
  public final Option<Boolean> rumble;
  public final Option<Boolean> intakeDown;
  public final Option<Boolean> intakeUp;
  public final Option<Boolean> tester;
  public final Option<Boolean> shootButton;
  public final Option<Boolean> homeIntake;
  public final Option<Boolean> visionCmd;
  public final Option<Boolean> outtakeIntake;
  public final Option<Boolean> outtakeShooter;
  public final Option<Boolean> enabled;
  public final Option<Double> motor1;
  public final Option<Boolean> hopperIsEmpty;
  public final Option<Boolean> alignNarrow;
  // public final Option<Double> motor2;
  // public final Option<String> lightState;
  public final Option<Boolean> intakeGoToDegrees;
  public final Option<Boolean> cruiseIntake;

  // Climber controls
  public final Option<Boolean> homeClimber;
  public final Option<Boolean> climberSequence;
  public final Option<Boolean> climberManualUp;
  public final Option<Boolean> climberManualDown;

  // Diagnostic controls
  public final Option<Boolean> diagShooter;
  public final Option<Boolean> diagHopper;
  public final Option<Boolean> diagIntakeRoller;
  public final Option<Boolean> diagIntakeUpDown;
  public final Option<Boolean> systemCheck;

  public ElasticMotorController() {
    super("Motor tester with elastic");

    cruiseIntake = createBooleanOption("INTAKE CRUISE (SAFE)", false);
    homeButton = createBooleanOption("HOME THE INTAKE", false);
    goToIdle = createBooleanOption("INTAKE IDLE", false);
    rumble = createBooleanOption("RUMBLE", false);
    intakeDown = createBooleanOption("INTAKE DOWN", false);
    intakeUp = createBooleanOption("INTAKE UP", false);
    tester = createBooleanOption("wtv", false);
    shootButton = createBooleanOption("shoot", false);
    homeIntake = createBooleanOption("homeintake", false);
    visionCmd = createBooleanOption("vision cmd", false);
    outtakeIntake = createBooleanOption("outtake intake", false);
    outtakeShooter = createBooleanOption("outtakeshooter", false);
    enabled = createBooleanOption("Enabled", false);
    motor1   = createNumberOption("Motor 1 Output", 0.0);
    hopperIsEmpty = createBooleanOption("Toggle lights tester", false);
    alignNarrow = createBooleanOption("narrow", false);
    intakeGoToDegrees = createBooleanOption("go to position cmd", false);
    // lightState = createStringOption("Light State", "IDLE");
    // motor2   = createNumberOption("Motor 2 Output", 0.0);

    homeClimber       = createBooleanOption("HOME CLIMBER", false);
    climberSequence   = createBooleanOption("CLIMBER SEQUENCE", false);
    climberManualUp   = createBooleanOption("CLIMBER MANUAL UP", false);
    climberManualDown = createBooleanOption("CLIMBER MANUAL DOWN", false);

    diagShooter        = createBooleanOption("DIAG SHOOTER", false);
    diagHopper         = createBooleanOption("DIAG HOPPER", false);
    diagIntakeRoller   = createBooleanOption("DIAG INTAKE ROLLER", false);
    diagIntakeUpDown   = createBooleanOption("DIAG INTAKE UPDOWN", false);
    systemCheck        = createBooleanOption("SYSTEM CHECK", false);

    OptionsRegistry.register(this);
  }
}
