package artisynth.tools.batchsim.conditions;

/**
 * A {@code Condition} is a conceptual representation of something that can be
 * either true or false (or equivalently met or not met) at any given point
 * time, but not both. However, the truth value of a {@code Condition} is
 * generally expected to change as time progresses.
 * <p>
 * From an implementation point of view, a class that implements this interface
 * represents a type or group of related conditions, and will usually be
 * implemented as an enumeration, where each enumerated value represents one
 * particular value of the associated {@code Condition} type. For example:
 * 
 * <pre>
 * <b>enum</b> BooleanLiteralCondition <b>implements</b> Condition {
 *    <b><i>TRUE</i></b>, <b><i>FALSE</i></b>;
 * }
 * </pre>
 * 
 * represents the Boolean literal condition type, which has only two condition
 * values: one condition that is invariably true, and one that is invariably
 * false.
 * 
 * @author Francois Roewer-Despres
 */
public interface Condition {

}
