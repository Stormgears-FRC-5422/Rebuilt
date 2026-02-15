package frc.robot.elastic;
import frc.utils.configfile.Option;
import frc.utils.configfile.OptionsController;
import frc.utils.configfile.OptionsRegistry;
public class ElasticMotorController extends OptionsController {
  public final Option<Boolean> homeButton;
  public final Option<Boolean> goToIdle;
  public final Option<Boolean> rumble;
  public final Option<Boolean> intakeDown;
  public final Option<Boolean> tester;
  public final Option<Boolean> shootButton;
  
  public ElasticMotorController() {
    super("Motor tester with elastic");
    homeButton = createBooleanOption("homeButton", false);
    goToIdle = createBooleanOption("send the intake to idle position", false);
    rumble = createBooleanOption("make the intake rumble", false);
    intakeDown = createBooleanOption("make intake go down", false);
    tester = createBooleanOption("wtv", false);
    shootButton = createBooleanOption("shoot", false);
    OptionsRegistry.register(this);
  }
}