package monologue;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * Interface for classes that can hold {@link Monologue} annotated fields for
 * {@link Monologue#setupMonologue} and {@link Monologue#logObj} to log.
 * 
 * This class also allows for an imperative way to log values with the {@link #log} methods.
 * 
 * Class fields that hold {@code Logged} objects should be final.
 * 
 * @see Monologue
 * @see Annotations.Log
 */
public interface Logged {
  static String getFullPath(Logged logged) {
    return Monologue.loggedRegistry.getOrDefault(logged, "notfound");
  }

  /**
   * Normally the name of {@code this} in the object tree is based off the field in the object the reference is stored in.
   * Overriding this method allows you to specify a different name for the object in the object tree.
   * 
   * If the returnd string has a '/' in it, the object will be placed in a subtable.
   *
   * @return The name of the object in the object tree.
   */
  public default String getOverrideName() {
    return "";
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default boolean  log(String key, boolean value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default boolean log(String key, boolean value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default int  log(String key, int value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default int log(String key, int value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default long  log(String key, long value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default long log(String key, long value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default float  log(String key, float value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default float log(String key, float value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default double  log(String key, double value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default double log(String key, double value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default String  log(String key, String value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default String log(String key, String value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default byte[]  log(String key, byte[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default byte[] log(String key, byte[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default boolean[]  log(String key, boolean[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default boolean[] log(String key, boolean[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default int[]  log(String key, int[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default int[] log(String key, int[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default long[]  log(String key, long[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default long[] log(String key, long[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default float[]  log(String key, float[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default float[] log(String key, float[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default double[]  log(String key, double[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default double[] log(String key, double[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default String[]  log(String key, String[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default String[] log(String key, String[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R extends StructSerializable> R log(String key, R value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default <R extends StructSerializable> R log(String key, R value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R extends StructSerializable> R[] log(String key, R[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default <R extends StructSerializable> R[] log(String key, R[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R> R log(String key, Struct<R> struct, R value) {
    return log(key, struct, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default <R> R log(String key, Struct<R> struct, R value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.putStruct(getFullPath(this) + "/" + key, struct, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R> R[] log(String key, Struct<R> struct, R[] value) {
    return log(key, struct, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param level The log level to log the value under.
   */
  public default <R> R[] log(String key, Struct<R> struct, R[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.putStructArray(getFullPath(this) + "/" + key, struct, value, level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log. (for enums, the names of the enums are logged)
   */
  public default <E extends Enum<?>> E log(String key, E value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log. (for enums, the names of the enums are logged)
   * @param level The log level to log the value under.
   */
  public default <E extends Enum<?>> E log(String key, E value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, value.name(), level);
    return value;
  }

  /**
   * Logs a value with the default log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log. (for enums, the names of the enums are logged)
   */
  public default <E extends Enum<?>> E[] log(String key, E[] value) {
    return log(key, value, LogLevel.DEFAULT);
  }

  /**
   * Logs a value with the specified log level.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log. (for enums, the names of the enums are logged)
   * @param level The log level to log the value under.
   */
  public default <E extends Enum<?>> E[] log(String key, E[] value, LogLevel level) {
    if (!Monologue.isMonologueReady(key) || Monologue.isMonologueDisabled()) return value;
    String[] names = new String[value.length];
    for (int i = 0; i < value.length; i++) {
      names[i] = value[i].name();
    }
    Monologue.ntLogger.put(getFullPath(this) + "/" + key, names, level);
    return value;
  }
}
