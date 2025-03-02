package monologue;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.LoggingTree.StaticObjectNode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class Monologue {
  private static boolean HAS_SETUP_BEEN_CALLED = false;
  static boolean IS_DISABLED = false;
  static EpilogueConfiguration config = new EpilogueConfiguration();

  private static final ArrayList<Runnable> prematureCalls = new ArrayList<>();
  private static final ArrayList<StaticObjectNode> trees = new ArrayList<>();
  
  /**
   * Monologue's global logger; can be used as a proxy where {@link LogLocal}
   * cannot be used(i.e commands, temporary objects, etc.)
   */
  public static final BackendOfGlobalLog GlobalLog = new BackendOfGlobalLog();
  
  public static class BackendOfGlobalLog implements LogLocal {
    BackendOfGlobalLog() {}
    
    public void setRootPath(String path) {
      var pathList = LogLocal.registry.get(this);
      if (pathList != null) pathList.clear();
      LogLocal.addNode(this, new LoggingTree.StaticObjectNode(path, this));
    }
    
    public void logMetadata(String key, String value) {
      config.backend.getNested("/Metadata/").log(key, value);
    }
    
    public void enableCommandLogging() {
      var scheduler = CommandScheduler.getInstance();
      var totalRuns = new HashMap<String, Integer>();
      var numInterrupts = new HashMap<String, Integer>();
      BiConsumer<Command, Boolean> handleFinish = (command, wasInterrupted) -> {
        String path = "commandStats/" + command.getName();
        log(path + "/isRunning", false);
        if (wasInterrupted) {
          log(path + "/numInterrupts", numInterrupts.merge(command.getName(), 1, Integer::sum));
        }
      };
      scheduler.onCommandInitialize(command -> {
        String path = "commandStats/"+ command.getName();
        log(path + "/requirements", command.getRequirements().toString());
        log(path + "/totalRuns", totalRuns.merge(command.getName(), 1, Integer::sum));
        log("commandStats/" + command.getName() + "/isRunning", true);
      });
      scheduler.onCommandFinish(command -> handleFinish.accept(command, false));
      scheduler.onCommandInterrupt(command -> handleFinish.accept(command, true));
    }
    
    public void logInit(String msg) {
      new Alert("Non-Urgent Alerts", msg, Alert.AlertType.kError).set(true);
    }
  }
  
  /**
   * Sets up monologue as disabled. This will cause log calls to be ignored.
   */
  public static void setupDisabled() {
    if (HAS_SETUP_BEEN_CALLED) {
      RuntimeLog.warn(
          "Monologue.setup() has already been called, further calls will do nothing");
      return;
    }
    HAS_SETUP_BEEN_CALLED = true;
    IS_DISABLED = true;
  }

  /**
   * Is the main entry point for the monologue library. It will interate over every member of the
   * provided Logged object and evaluated if it should be logged to the network tables or to a file.
   *
   * <p>Will also recursively check field values for classes that implement LogLocal and log those as
   * well.
   *
   * @param robot the root Logged object to log
   * @apiNote Should only be called once, if another {@link LogLocal} tree needs to be created use
   *     {@link #logTree(Object, String)} for additional trees
   */
  public static <R extends TimedRobot & LogLocal> void setup(R robot, EpilogueConfiguration epilogueConfig) {
    if (HAS_SETUP_BEEN_CALLED) {
      RuntimeLog.warn(
          "Monologue.setup() has already been called, further calls will do nothing");
      return;
    }
    HAS_SETUP_BEEN_CALLED = true;
    Monologue.config = epilogueConfig;
    RuntimeLog.logger = config.backend;
    logTree(robot, config.root);
    GlobalLog.setRootPath(config.root);
    prematureCalls.forEach(Runnable::run);
    robot.addPeriodic(Monologue::updateAll, robot.getPeriod());
    System.gc();
    RuntimeLog.info("Monologue.setup() finished");
  }
  
  public static boolean isDebugMode() {
    return config.minimumImportance == Logged.Importance.DEBUG;
  }
  
  static void logTree(Object loggable, String path) {
    if (notSetup())
      throw new IllegalStateException(
          "Tried to use Monologue.logTree() before using a Monologue setup method");

    if (path == null || path.isEmpty()) {
      RuntimeLog.warn("Invalid path for Monologue.logTree(): " + path);
      return;
    } else if (path.equals("/")) {
      RuntimeLog.warn("Root path of / is not allowed for Monologue.logTree()");
      return;
    }
    RuntimeLog.info(
        "Monologue.logTree() called on " + loggable.getClass().getName() + " with path " + path);

    StaticObjectNode node = new LoggingTree.StaticObjectNode(path, loggable);
    Eval.exploreNodes(Eval.getLoggedClasses(loggable.getClass()), node);
    LogLocal.addNode(loggable, node);

    trees.add(node);

    updateAll();
  }

  /**
   * Updates all the loggers, ideally called every cycle.
   *
   * @apiNote Should only be called on the same thread monologue was setup on
   */
  static void updateAll() {
    if (notSetup())
      RuntimeLog.warn("Called Monologue.updateAll before Monologue was setup");
    for (StaticObjectNode tree : trees) {
      tree.log(null);
    }
  }

  static void prematureLog(Runnable runnable) {
    prematureCalls.add(runnable);
  }

  /**
   * Checks if the Monologue library isn't setup.
   *
   * @return false if Monologue has been setup, true otherwise
   */
  static boolean notSetup() {
    return !HAS_SETUP_BEEN_CALLED;
  }
}
