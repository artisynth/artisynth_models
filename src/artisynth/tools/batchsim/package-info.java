/**
 * Package implementing the <i>Batch Simulation Framework</i>. <br>
 * <br>
 * <h1>1. INTRODUCTION</h1> The Batch Simulation Framework, also known as
 * BatchSim, is a framework that allows a large number of simulations to be
 * automatically performed using any arbitrary
 * {@link artisynth.core.workspace.RootModel RootModel} subclass (BatchSim is
 * model-agnostic) by accessing model attributes through {@code ArtiSynth}'s
 * {@link maspack.properties.Property Property} mechanism. See the <a href=
 * "http://artisynth.magic.ubc.ca/pmwiki.php?n=Documentation.MaspackRefManual">
 * Maspack Reference Manual</a> for details on {@code Properties}.
 * <p>
 * The basic idea is to specify a list of {@code Properties}, and, for each
 * {@code Property}, to specify a set of values that it should take. This
 * property-value-set specification is known as a
 * {@link artisynth.tools.batchsim.PropertySpecification property
 * specification}. Thus, the user input to BatchSim consists of a list of
 * property specifications. BatchSim iterates through this list in a
 * combinatorial fashion (explained later), and performs one simulation per
 * resulting combination by setting, for each property specification, the given
 * {@code Property} to each value in its value set in turn. Thus, the
 * user-specified {@code Property}({@code ies}) serve as variable input(s) into
 * the model, and a simulation is performed for each such input, allowing some
 * variable-dependent output(s) of the model to be recorded.
 * <p>
 * Alternatively, a {@code Property} can be specified to follow a particular
 * <a href="https://en.wikipedia.org/wiki/Probability_distribution"> parametric
 * probability distribution</a>, and BatchSim will perform simulations, where,
 * for each simulation, the given {@code Property} is set to a random value
 * sampled from its probability distribution. In this case, a {@code Property}
 * is no longer just a variable, but rather a true
 * <a href="https://en.wikipedia.org/wiki/Random_variable">random variable</a>,
 * and its property specification is probabilistic, rather than combinatorial.
 * <p>
 * BatchSim's use of the very general and widely-used {@code Property} mechanism
 * allows it to work effectively with any {@code RootModel} subclass.
 * Frequently, the input parameter that is to vary with each simulation is
 * already a {@code Property}, either of the {@code RootModel} itself, or else
 * of one of its subcomponents (usually a
 * {@link artisynth.core.modelbase.ModelComponent ModelComponent}). In the
 * unlikely case that an input parameter is not already a {@code Property} of
 * some {@code ModelComponent} or of the {@code RootModel}, a {@code Property}
 * can readily be defined to "wrap" said input parameter so as to be accessible
 * via the usual {@code Property} mechanism. Refer to the {@code Maspack
 * Reference Manual} for details on adding custom {@code Properties} to any
 * class, including a {@code RootModel} subclass.
 * <p>
 * An explicit assumption made throughout BatchSim is that the value set (or
 * probability distribution) of each {@code Property} is specified prior to
 * performing any simulations. Formally, the assumption states that the result
 * or output of any one simulation cannot serve to indicate, in whole or in
 * part, the input value of any {@code Property} in any subsequent simulation. A
 * direct consequence of this assumption is that each simulation is entirely
 * independent of each other simulation. As such, performing multiple
 * simulations becomes an
 * <a href="https://en.wikipedia.org/wiki/Embarrassingly_parallel">
 * embarrassingly parallel</a> problem.
 * <p>
 * BatchSim exploits this by adhering to the manager-worker model (a special
 * case of the
 * <a href="https://en.wikipedia.org/wiki/Client-server_model">client-server
 * model</a>). By following this design pattern, BatchSim not only achieves
 * terrific load balancing, but also enables the distribution of the "manager"
 * and the "workers" over several networked computers. As such, BatchSim is a
 * general-purpose framework for automatically performing a large number of
 * {@code ArtiSynth} model simulations, and it can easily be made to do so in a
 * distributed and parallel fashion, as is usually desired.
 * <p>
 * Specifically, one {@link artisynth.tools.batchsim.BatchManager manager}
 * process creates a number of "tasks", where each task represents a simulation
 * to perform. The manager then places these simulation tasks in a "bag of
 * tasks". Next, a certain number of
 * {@link artisynth.tools.batchsim.BatchWorkerBase worker} processes are
 * created. Each worker requests a task from the manager, completes the task (by
 * performing the simulation the task describes), records user-specified
 * simulation results as output, and repeats until the manager informs the
 * worker that there are no more simulation tasks to complete. The workers
 * complete their tasks independently and in parallel, allowing the entire
 * problem (running a set of simulations) to be solved in less time.
 * <p>
 * BatchSim includes a number of convenience methods for recording certain kinds
 * of simple, generic simulation output, but custom output and output formats
 * can be specified by the user (see
 * {@link artisynth.tools.batchsim.BatchWorkerBase BatchWorkerBase} for
 * details). <br>
 * <br>
 * <h1>2. SIMULATION TASKS AND PROPERTY SPECIFICATIONS</h1> A simulation task
 * consists of a list of property-value pairs. The property is either the name
 * of a {@code Property} or else a property path (the path from the
 * {@code RootModel} to one of its {@code ModelComponent}s, followed by
 * `{@code :}', followed by the name of a property of that component). For
 * {@link maspack.properties.CompositeProperty CompositeProperties},
 * sub-{@code Properties} can be referenced by separating the name of the
 * {@code CompositeProperty} and its sub-{@code Properties} with `{@code .}'.
 * The value in each property-value pair is a textual (i.e. string)
 * representation of a value that can be assigned to the associated
 * {@code Property}. The format of this string must be such that the
 * {@link maspack.properties.PropertyInfo#scanValue(maspack.util.ReaderTokenizer)
 * PropertyInfo.scanValue(ReaderTokenizer)} method can scan it.
 * <p>
 * The manager creates the property-value pairs that appear in each simulation
 * task from the list of property specifications mentioned in the introduction.
 * There are two kinds of property specifications: <b>combinatorial</b>
 * specifications and <b>probabilistic</b> specifications. <br>
 * <h2>2.1. Combinatorial Property Specifications</h2> Here, a particular
 * {@code Property} is associated with a set of values, and each value is
 * assigned to the {@code Property} in turn. Other {@code Properties} are
 * associated with different value sets in their own combinatorial
 * specification. The manager iterates through the values in the specifications
 * so as to perform an exhaustive search of the input space: each possible
 * combination of property-value pairs for all combinatorial specifications
 * combined is examined. Each simulation tasks consists of exactly one such
 * combination. More formally, say we have {@code N} such combinatorial
 * specifications. Then, the general exhaustive-search/combinatorial algorithm
 * is as follows:
 * 
 * <pre>
 * for value1 in specification1.valueSet()
 *    for value2 in specification2.valueSet()
 *       // ...
 *          for valueN in specificationN.valueSet()
 *             createSimulationTask(value1, value2, ..., valueN)
 *          endFor
 *       // ...
 *    endFor
 * endFor
 * </pre>
 * 
 * Mathematically speaking, one simulation task is performed for each element in
 * the set that results from taking the
 * <a href="https://en.wikipedia.org/wiki/Cartesian_product"> Cartesian
 * product</a> of all given value sets.
 * 
 * <h2>2.2. Probabilistic Property Specifications</h2> Here, a particular
 * <u>numeric</u> {@code Property} is associated with a vector of parametric
 * probability distributions, with the size of the probability distribution
 * vector equal to the size of the {@code Property} vector. For a scalar numeric
 * {@code Property}, the probability distribution vector should contain a single
 * entry. Other {@code Properties} are associated with different probabilistic
 * distribution vectors in their own probabilistic specification. If <b>only</b>
 * probabilistic specifications are given, the manager creates simulation tasks
 * by applying the Monte Carlo method. That is, for each probabilistic
 * specification, all the distributions listed in its associated probability
 * distribution vector are sampled, and a random-value vector is formed from the
 * sampled values. These sampled vectors are then assembled into one simulation
 * task (thus forming one Monte Carlo simulation task), and the process is
 * repeated {@code M} times, where {@code M} is specified by the user. Formally,
 * the algorithm is as follows:
 * 
 * <pre>
 * for i in 1 to M
 *    vectors = new List()
 *    for specification in specificationsList
 *       sampledVector = new Vector()
 *       for distribution in specification.distributionVector()
 *          sampledVector.add(distribution.sample())
 *       endFor
 *       vectors.add(sampledVector)
 *    endFor
 *    createSimulationTask(vectors)
 * endFor
 * </pre>
 * 
 * <i>A note on running hybrid combinatorial/probabilistic simulations:</i><br>
 * If at least one probabilistic specification and at least one combinatorial
 * specification are given, the combinatorial algorithm described above is used,
 * but augmented with a check for the specification's type. If the specification
 * is probabilistic, it is treated as if its distribution vector was a single
 * combinatorial value (whose value just happens to be different with each
 * combination). The end result is a hybrid algorithm between
 * exhaustive-search/combinatorial and Monte Carlo, which is described by the
 * following algorithm:
 * 
 * <pre>
 * iterator = specificationsList.iterator()
 * while iterator.hasNext()
 *    specification1 = iter.next()
 *    if specification1.type() is COMBINATORIAL
 *       for value in specification1.valueSet()
 *          specification2 = iterator.next()
 *          // ...
 *             createSimulationTask(value, ...)
 *          // ...
 *       endFor
 *    else // specification1.type() == PROBABILISTIC
 *       sampledVector = new Vector()
 *       for distribution in specification1.distributionVector()
 *          sampledVector.add(distribution.sample())
 *       endFor
 *       specification2 = iterator.next()
 *       // ...
 *          createSimulationTask(sampledVector, ...)
 *       // ...
 *    endIf 
 * endWhile
 * </pre>
 * 
 * <h2>2.3. Property Specifications: Format</h2> A number of combinatorial
 * and/or probabilistic property specifications are typically listed in an input
 * file, which is then provided to the manager. The input file format for both
 * property specifications types is as follows:
 * <ul>
 * <li>Each <i>combinatorial</i> specification must adhere to the following
 * general format:<br>
 * <code>&nbsp;&nbsp;&nbsp;&lt;property_path&gt; = &lt;value_set&gt;</code><br>
 * where {@code <property_path>} must be a valid {@code ArtiSynth} property path
 * except for an optional component identifier set extension provided by
 * BatchSim that allows multiple components to be grouped into a single
 * specification (see below); and where {@code <value_set>} must be of the form:
 * <br>
 * <code>&nbsp;&nbsp;&nbsp;{<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;delim&gt;&lt;val1&gt;&lt;delim&gt;<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;delim&gt;&lt;val2&gt;&lt;delim&gt;<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;...<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;delim&gt;&lt;valN&gt;&lt;delim&gt;<br>
 * &nbsp;&nbsp;&nbsp;}</code> <br>
 * where {@code <delim>} is a chosen value delimiter character (`{@code %}' by
 * default). The entire string between two consecutive occurrences of {@code 
 * <delim>} is taken to represent a single, syntacticly-valid value for the
 * associated {@code Property}.</li>
 * <li>Some examples of valid combinatorial property specifications (when the
 * delimiter character is the default `{@code %}'):
 * <ol>
 * <li><code>"models/0/component/1:someProp.someSubProp" = {%1% %2% %3%}</code>
 * </li>
 * <li><code>"color" = {%"red"% %"green"%}</code></li>
 * <li><code>"someVectorProp" = {<br>
 * &nbsp;&nbsp;%[1 2 3]%<br>
 * &nbsp;&nbsp;%[4 5 6]%<br>
 * }</code></li>
 * </ol>
 * </li>
 * <li>Each <i>probabilistic</i> specification must adhere to the following
 * general format:<br>
 * <code>&nbsp;&nbsp;&nbsp;&lt;property_path&gt; ~ &lt;distribution_vector&gt;</code><br>
 * where {@code <property_path>} must be a valid {@code ArtiSynth} property path
 * except for an optional component identifier set extension provided by
 * BatchSim that allows multiple components to be grouped into a single
 * specification (see below); and where {@code <distribution_vector>} must be of
 * the form:<br>
 * <code>&nbsp;&nbsp;&nbsp;[<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;Name_of_distribution1&gt;(&lt;parameter1_value&gt;, ..., &lt;parameterN_value&gt;)<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;Name_of_distribution2&gt;(&lt;parameter1_value&gt;, ..., &lt;parameterN_value&gt;)<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;...<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;Name_of_distributionN&gt;(&lt;parameter1_value&gt;, ..., &lt;parameterN_value&gt;)<br>
 * &nbsp;&nbsp;&nbsp;]</code> <br>
 * where each {@code <Name_of_distributionX>} is the <b>non-quoted</b> string
 * name of a probability distribution. To be valid, the string name must match
 * {@link artisynth.tools.batchsim.DistributionSampler.Distribution#toString()
 * Distribution.toString()} for some
 * {@link artisynth.tools.batchsim.DistributionSampler.Distribution
 * Distribution} value. Following the distribution name, a comma-separated list
 * of parameter values (integers or doubles) enclosed in one pair of parentheses
 * should appear. The values must appear in the same order, and be in the same
 * quantity, as specified by the corresponding {@code Distribution}, and each
 * value must be a valid number for that particular parameter of that
 * {@code Distribution}. The number of {@code Distributions} listed between the
 * pair of square brackets must match the size of the associated
 * {@code Property}'s data type. For scalar-valued {@code Properties}, a single
 * {@code Distribution} should be listed. For vector-valued {@code Properties},
 * the number of {@code Distributions} listed should match the vector's
 * size.</li>
 * <li>In the probabilistic specification, {@code ~} is analogous to the
 * notation in Probability and Statistics that if a random variable
 * <code><b><i>X</i></b></code> is distributed according to the parametric
 * probability distribution <code><i>D</i></code> with, say, 2, parameters
 * <code><i>a</i></code> and <code><i>b</i></code>, then we write
 * <code><i><b>X</b> ~ D(a, b)</i></code>. Here, we augment this notation by
 * using <code><b><i>X</i></b></code> (the {@code Property}) to represent a
 * <i>vector</i> of random variables, and we specify the parametric probability
 * distribution according to which each entry of <code><b><i>X</i></b></code> is
 * distributed (using the distribution vector that follows {@code ~}).</li>
 * <li>Some examples of valid probabilistic property specifications:
 * <ol>
 * <li>
 * <code>"models/0/component/1:someScalarProp" ~ [ Normal(20.0, 1.5) ]</code>
 * </li>
 * <li><code>"colorRGB" ~ [ Uniform(0, 1) Uniform(0, 1) Uniform(0, 1) ]</code>
 * </li>
 * <li><code>"someVectorPropWithConstant2ndEntry" ~ [<br>
 * &nbsp;&nbsp;T(2)<br>
 * &nbsp;&nbsp;Uniform(1, 1)<br>
 * &nbsp;&nbsp;Normal(0, 1)<br>
 * ]</code></li>
 * </ol>
 * </li>
 * </ul>
 * <h2>2.4. Property Specifications: Syntax</h2> Some notes on the input file
 * syntax (in terms of tokens) of property specifications follow. Unless
 * otherwise indicated, syntax rules apply to both types of property
 * specifications.
 * <ul>
 * <li>The `{@code #}' token begins a comment. All characters between it and the
 * next end-of-line character are ignored.</li>
 * <li>All tokens in any specification can be separated by an arbitrary number
 * of whitespace characters.</li>
 * <li>The {@code <property_path>} token <i>must</i> be double-quoted.</li>
 * <li>The {@code <property_path>} token must be followed by a `{@code =}' token
 * (for combinatorial specifications), or by a `{@code ~}' token (for
 * probabilistic specifications).</li>
 * <li>The `{@code =}' token must be followed by exactly one pair of curly
 * braces (for combinatorial specifications). The `{@code ~}' token must be
 * followed by exactly one pair of square brackets (for probabilistic
 * specifications).</li>
 * <li>The list of {@code <valX>} tokens in the value set of every combinatorial
 * specification must all lie within the pair of curly braces. The individual
 * values must be enclosed in a pair of delimiter characters (which is the
 * `{@code %}' token by default). The following examples are all syntactically
 * valid and semantically equivalent <i>combinatorial</i> property
 * specifications:
 * <ol>
 * <li><code>"models:prop" = { %val1% %val2% }</code></li>
 * <li><code>"models:prop" = {%val1% %val2%}</code></li>
 * <li><code>"models:prop"={%val1%%val2%}</code></li>
 * <li><code>"models:prop" = {<br>
 * &nbsp;&nbsp;%val1%<br>
 * &nbsp;&nbsp;%val2%<br>
 * }</code></li>
 * </ol>
 * </li>
 * <li>Note that the value set of a combinatorial specification can be empty:
 * `<code>{}</code>'. In this case, but <u>only</u> in this case, the associated
 * {@code Property} will be assigned the value <b>{@code null}</b> in all
 * simulation tasks.</li>
 * <li>Note that whitespace appearing in any double-quoted string, whether in
 * the {@code <property_path>} token, or in a value that is a string, will be
 * considered part of the string token, unless it is an end-of-line character,
 * which will cause the string to be split into multiple word tokens. This
 * applies to value strings delimited by the delimiter character as well. One
 * exection to this rule is that non-end-of-line whitespace appearing within a
 * component identifier set of {@code <property_path>} token is ignored (see
 * below).</li>
 * <li>Again, the `{@code %}' tokens appearing in some of the preceding
 * combinatorial specification examples represents the default value delimiter
 * character for a combinatorial specification value set. It can be changed to
 * another character by supplying the appropriate option to the manager. Note
 * that some characters are not allowed as the delimiter character. These
 * include any character that already has special syntactic significance, such
 * as `{@code =}' or `{@code "}', for example, and also digit and whitespace
 * characters.</li>
 * <li>The syntax for the distribution vector of a probabilistic specification
 * has already been described in "2.3. Property Specifications: Format", since
 * the syntax and the format are essentially identical for probabilistic
 * specifications. Note simply in addition that each token in the distribution
 * vector can be separated by an arbitrary amount of whitespace.</li>
 * </ul>
 * <h2>2.5. Property Specifications: Component Identifier Set</h2>
 * <ul>
 * <li>Within the {@code <property_path>} token, a special set notation can be
 * used as a shorthand to group multiple subcomponents of a particular component
 * into a single specification. This is useful if some or all subcomponents
 * would otherwise be specified as having the same value set (for combinatorial
 * specifications) or the same distribution vector (for probabilistic
 * specifications).</li>
 * <li>Within the path, instead of a single component name or component number
 * (collectively called a component identifier), a <u>set</u> of identifiers can
 * be specified (within a pair of curly braces). This set will be "expanded" in
 * place by creating one (implicit) specification for each component identifier
 * in the set. For example, the {@code <property_path>} token<br>
 * <code>
 * &nbsp;&nbsp;&nbsp;"models/{mySubComponent1 mySubComponent2}:prop"
 * </code><br>
 * will be expanded into the following 2 tokens:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/mySubComponent1:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/mySubComponent2:prop"</code></li>
 * <li>This expansion works regardless of the property specification type: the
 * value set (for combinatorial specifications) or distribution vector (for
 * probabilistic specifications) is copied (re-instantiated) for each expanded
 * {@code <property_path>} token.</li>
 * <li>The individual identifiers in a component identifier set must be
 * separated by an arbitrary number of non-end-of-line whitespace characters.
 * Note that since a component identifier set appears within the double-quoted
 * string of a {@code <property_path>} token, it cannot contain `{@code "}'
 * characters anywhere. Further, since components cannot be identified without
 * an identifier, a component identifier set cannot be empty.</li>
 * <li>Since the unique number of any ArtiSynth subcomponent is an integer, an
 * <i>inclusive</i> <b>range</b> of such integers can be provided as an
 * additional shorthand. For example, the {@code <property_path>} token<br>
 * <code>
 * &nbsp;&nbsp;&nbsp;"models/{[0-2] [24-25]}:prop"</code> <br>
 * will be expanded into the following 5 {@code <property_path>} tokens:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/0:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/1:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/2:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/24:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/25:prop"</code></li>
 * <li>As always, an arbitrary number of whitespace characters can separate the
 * individual characters in a range.</li>
 * <li>Note that the first integer specified in a range must be less than or
 * equal to the second integer in the range, but separate ranges need not be
 * listed in increasing order.</li>
 * <li>A component identifier set will be expanded in place (such that the
 * substring delimited by the curly braces will be exactly replaced by each
 * identifier listed in the set. For example, the {@code <property_path>}
 * token<br>
 * <code>
 * &nbsp;&nbsp;&nbsp;"models/comp{&nbsp;&nbsp;&nbsp;[0-2] [24-25]&nbsp;}onent:prop"
 * </code><br>
 * will be expanded into the following 5 {@code <property_path>} tokens:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp0onent:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp1onent:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp2onent:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp24onent:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp25onent:prop"</code><br>
 * and the {@code <property_path>} token<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/{ left right top }_comp:prop"</code><br>
 * will be expanded into the following 3 specifications:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/left_comp:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/right_comp:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/top_comp:prop"</code></li>
 * <li>Note that identifiers in a component identifier set can in fact be
 * component paths or path fragments as well. For example, the
 * {@code <property_path>} token<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/{comp0/1 comp2/3}:prop"</code><br>
 * will be expanded into the following 2 {@code <property_path>} tokens:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp0/1:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp2/3:prop"</code></li>
 * <li>Finally, note that multiple component identifier sets can be specified
 * within a single {@code <property_path>} token, but that component identifier
 * sets cannot be nested. For example, the {@code <property_path>} token<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp{0 1}/{2 3}:prop"</code><br>
 * will be expanded into the following 4 {@code <property_path>} tokens:<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp0/2:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp0/3:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp1/2:prop"</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp1/3:prop"</code><br>
 * but the {@code <property_path>} token<br>
 * <code>&nbsp;&nbsp;&nbsp;"models/comp{[0-1] {2 3}}:prop"</code><br>
 * is <b><u>not</u></b> valid and will not be expanded.</li>
 * </ul>
 * <h2>2.6. Property Specifications: Decorators</h2>
 * <ul>
 * <li>Sometimes, we may want to specify a property specification as
 * probabilistic, but then pre-sample from its distribution vector a certain
 * number of times, say {@code k}, to get {@code k} fixed values from those
 * distribution. Then, we may want to treat these {@code k} values as though
 * they were values in a combinatorial value set. In other words, the property
 * specification is specified as probabilistic, but we sample from it {@code k}
 * times, and then treat the resulting values as the {@code k} values of the
 * value set of a <i>combinatorial</i> property specification. To have
 * repeatable results, consider fixing the seed of the random number generator
 * of the {@link artisynth.tools.batchsim.manager.BatchManager BatchManager}.</li>
 * <li>Similarly, we may want to specify as property specification as
 * combinatorial, but instead of iterating deterministically through its value
 * set to created specification tasks, we may want to randomly sample from this
 * set. Furthermore, we may want each value to have a different probability of
 * being sampled. In other words, the property specification is specified as
 * combinatorial, but we treat its value set as the support of a discrete
 * probability distribution, and then sample from this distribution as if the
 * property specification was <i>probabilistic</i>.</li>
 * <li>The syntax is as follows:
 * <ol>
 * <li>{@code @PROB <combinatorial_property_specification>} to have uniform
 * sampling of the value set.</li>
 * <li>{@code @PROB(p1, p2, ..., pN) <combinatorial_property_specification>} to
 * have value-specific probabilities. Here, the sum of pi, i = 1..N should equal
 * 1, and all should be greater than 0. The first form is just shorthand for
 * this form where pi = 1/N for all i. Note that the number of values in the
 * value set <b>must</b> match the number of arguments to the {@code @PROB}
 * decorator.</li>
 * <li>{@code @COMB(k) <probabilistic_property_specification>} to sample the
 * distribution vector of the property specification {@code k} times.</li>
 * </ol>
 * </li>
 * <li>
 * Sometimes it may be useful to have a property specification that exists
 * solely for triggering control statements (see Section 2.7). Since property
 * specifications that don't correspond to a real {@code Property} in the
 * {@code RootModel} subclass used in the simulation results in BatchSim
 * throwing a runtime error, the only way to use such a phony specification
 * would be to add a new {@code Property} to the {@code RootModel} subclass,
 * which is not very elegant. Instead, a third kind of decorator exists, the
 * {@code @PHONY} decorator, which simply flags the property specification as
 * phony. The manager never passes any phony property specifications to the
 * workers, which resolves the problem. 
 * </li>
 * <li>
 * At the present time, it is not possible to add multiple {@code @PROB} or
 * {@code @COMB} decorators to a property specification. However, any property
 * specification (whether decorated or not) can be <b>prepended</b> by at most
 * one {@code @PHONY} decorator.
 * </li>
 * </ul>
 * <h2>2.7. Control Statements</h2>
 * Property Specifications can be grouped by (currently) 3 control structures.
 * This makes a sort of mini programming language, which is called the Property
 * Specification Language, or PSL. The 3 structures are <u>skip statements</u>,
 * <u>redefinition statements</u>, and <u>Jython code blocks</u>. These structures
 * are useful for removing certain combinations of values that may be 'illegal' and
 * would otherwise be discarded as invalid simulations in post-processing. The power
 * of using these structures comes from the fact that they save valuable computation
 * time by not running simulations that will end up being discarded anyways. It is
 * also helpful, because discarding specific pieces of data later on may be
 * non-trivial, and may require manual intervention (although this depends on how the
 * output data is recorded).
 * <ul>
 * <li><u>Skip statements</u>. A skip statement is a list of <b>combinatorial</b>
 * property specifications, each of which has to be defined at some time earlier
 * in the input file. A skip statement <i>skips</i> all simulations where each
 * of the listed property specifications has a current value equal to one of the
 * listed values in the value set found within the skip statements. That is,
 * multiple specifications listed together form a logical AND and within each one,
 * the value set forms a logical OR. This is called a match. The syntax of a skip
 * statement is the keyword "skip" followed by a list of <b>undecorated,
 * combinatorial</b> property specifications (called the skip block), followed by
 * the keyword "end". For example:<br>
 * <code>&nbsp;&nbsp;&nbsp;"prop1" = {%0% %1% %2%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4% %5%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;skip</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop1" = {%1% %2%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;end</code><br>
 * will skip all simulations where ("prop1" has value 1 OR 2) AND
 * ("prop2" has value 3 OR 4), but allow all others (i.e. the pairs (0, 3), (0, 4),
 * (0, 5), (1, 5), and (2, 5) are all allowed).</li>
 * <li><u>Redefinition statements</u>. Also known as "redef" statements. These are
 * composed of two blocks. The syntax is the keyword "redef" followed by a list of
 * property specifications, which may be combinatorial or probabilistic, and
 * decorated or not (called the redef block), followed by the keyword "when", followed
 * by a list of <b>undecorated, combinatorial</b> property specifications (called the
 * when block), followed by the keyword "end". The when block is similar to the skip
 * block of the skip statement, except that instead of skipping simulation tasks that
 * match the block, it triggers the redefinition of some other property specification(s).
 * By redefinition, we mean that the property path stays the same, and so refers to the
 * same property in the model, but the specification changes. The change can be anything:
 * adding a decorator, changing the values in the value set, changing the probability
 * distributions in a distribution vector, changing the size of the value set or
 * distribution vector, changing the property specification from combinatorial to
 * probabilistic, or anything else. The redef block contains a list of these redefined
 * property specifications. This is just a shorthand: in reality there is one separate
 * and completely independent redefinition for each property specification listed in the
 * redef block. As such, it would be equivalent to having one redef with the same when
 * block for each property specification listed in the redef block. One catch: <b>every</b>
 * property specification listed in the redef block must be originally defined <b>after</b>
 * every specification listed in the when block. In other words, we can think of the
 * contents of the input file as a directed graph where each property specification is a
 * node, and where there is a directed edge from one property specification to another if
 * the former appears immediately before the latter in this file. Then, a topological sort
 * of the resulting property specification directed acyclic graph (DAG) would place the
 * property specifications in the same order as they appear in the input file. With this
 * in mind, we can think of a redef statement as introducing several more directed edges
 * to the property specification DAG. Each such edge starts at a property specification in
 * the when block and ends at a property specification in the preceding redef block. The
 * redef statement, then, is valid if, and only if, the property specification graph that
 * results from the introduction of these new edges is still a DAG (i.e. has no cycles).
 * This is done to prevent cyclical redefinitions, which are usually hard to get right,
 * and often cause infinite loops unless extreme care is taken to avoid them. Redef
 * statements can be useful in the following way (for example):<br>
 * <code>&nbsp;&nbsp;&nbsp;"prop1" = {%LOW% %HIGH%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"prop2" = {} # I.e. prop2 is set to null.</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"prop3" = {%value_for_low% %value_for_high%} # Or could be set to null.</code><br>
 * <code>&nbsp;&nbsp;&nbsp;redef</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop2" = {%1% %2%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop3" = {%value_for_low%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;when</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop1" = {%LOW%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;end</code><br>
 * <code>&nbsp;&nbsp;&nbsp;redef</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop2" = {%9% %10%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop3" = {%value_for_high%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;when</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop1" = {%HIGH%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;end</code><br>
 * "prop1" acts as a flag which makes some other properties, namely "prop2" and "prop3",
 * have values that are either 'low' or 'high', and effectively skips simulations where
 * one has a 'low' value while the other has a 'high' value, which we would want to
 * discard as invalid. In this way, redef and skip statements are equivalent: they are
 * really just different ways of expressing the same thing, and one can always be defined
 * in terms of the other. However, the logic may be much more complicated to express with
 * one of the statements compared to the other, so both are available, for convenience.
 * </li>
 * <li><u>Jython code blocks</u>. Jython code blocks are different from skip and redef
 * statements in that they cannot be stand-alone statements, and must instead occur within
 * a skip statement or the when block of a redef statement. They act as an additional way
 * in which skip or when matches can be detected. This can be useful, for example, to
 * match conditional on the value of a probabilistic property specification. In fact,
 * these code blocks can entirely replace the combinatorial matching described above. The
 * advantage of the combinatorial matching, however, is in terms of a convenient syntax,
 * <b>and noticeably faster computation time</b> on the part of the manager (the workers
 * are unaffected). The syntax of a Jython code block is the keyword "jython" followed by
 * a list of arbitrary Jython code line, each of which must be <b>prepended by a `$'</b>,
 * followed by the keyword "end". Within the Jython code, 2 special functions are exposed.
 * The {@code get()} function takes a string representing a property path, and returns the
 * current value of the property specification with that property path (as a string), or
 * {@code None} if the value is not set. The {@code return_value()} function takes a
 * boolean and returns nothing. It is a callback that returns the value of the code block
 * to the manager. That is, if the code block should trigger a match (either in a skip
 * statement or a when block), then {@code return_value(True)} should be called. Otherwise,
 * {@code return_value(False)} should be called. Calling {@code return_value()} multiple
 * times within a single Jython code block is not permitted. Note that if a skip statement
 * or when block contains both Jython code blocks and combinatorial property specifications,
 * then all of them must match for a global match to be established. For example:<br>
 * <code>&nbsp;&nbsp;&nbsp;"prop1" = {%0% %1% %2%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4% %5%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;skip</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;jython</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$prop1_val = get("prop1")</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$return_value(prop1_val in ["1", "2"])</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;end</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;end</code><br>
 * is identical to the example of the skip statement shown above, <b>except that it will be
 * computationally slower</b>. In addition, one-liners can be specified by appending an
 * additional `$' to the end of the Jython code. For example, the following is identical to
 * the previous example:<br>
 * <code>&nbsp;&nbsp;&nbsp;"prop1" = {%0% %1% %2%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4% %5%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;skip</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;jython $return_value(get("prop1") in ["1", "2"])$ end</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"prop2" = {%3% %4%}</code><br>
 * <code>&nbsp;&nbsp;&nbsp;end</code><br>
 * </li>
 * </ul>
 * Note that very little checking is done to ensure valid use of these control structures,
 * because they can interact in complex and very subtle ways that are hard to verify ahead
 * of time. In particular, no checks are made for multiple redefs that are triggered at
 * the same time, or when skip and redefs or skips and other skips are incompatible. In
 * particular, multiple redefs can "cascade", where one redef is triggered, which
 * redefines some other property, which later (due to this definition) triggers another
 * redef. Although this is valid, and even encouraged in many cases, no checks are made to
 * ensure nothing is done incorrectly; instead, task creation proceeds assuming everything
 * will work out, and only if an error is detected during task creation will a meaningful
 * error message be given.<br>
 * <br>
 * <h1>3. USING THE BATCH SIMULATION FRAMEWORK</h1>
 * <h2>3.1. Prerequisites for using BatchSim</h2>
 * <ul>
 * <li>A {@code RootModel} subclass with appropriate {@code Properties} with
 * which to perform simulations. Refer to the <a href=
 * "http://artisynth.magic.ubc.ca/pmwiki.php?n=Documentation.ModelingGuide">
 * ArtiSynth Modeling Guide</a> for details.</li>
 * <li>(Optional) A {@link artisynth.tools.batchsim.BatchWorkerBase worker}
 * subclass to customize simulation stop conditions and output, and (optionally)
 * to customize simulation input (for example, by adding an
 * {@link artisynth.core.probes.InputProbe InputProbe}). Refer to the
 * {@link artisynth.tools.batchsim.BatchWorkerBase BatchWorkerBase} JavaDocs for
 * details. Alternatively, a "default" worker subclass can be used:
 * {@link artisynth.tools.batchsim.SimpleTimedBatchWorker
 * SimpleTimedBatchWorker}, which has a single time stop condition (stopping the
 * simulation after a certain amount of time has elapsed), records the end-state
 * of the model in binary waypoint files, and does some simple logging.</li>
 * <li>An input file listing property specifications (the input data needed for
 * performing the simulations). Refer to "2. SIMULATION TASKS AND PROPERTY
 * SPECIFICATIONS" in this document for details.</li>
 * <li>(Optional) A very short {@code Jython} script for driving the worker
 * subclass. This is required only if a custom worker subclass is being used, as
 * otherwise the {@code batchDriver.py} script in this package can be used to
 * drive the {@code SimpleTimedBatchWorker}.</li>
 * <li>(Optional) A convenient place to store the worker subclass, the input
 * file, the {@code Jython} script, and the output file(s), such as a
 * sub-package of the {@code RootModel} subclass's package in
 * {@code artisynth_projects}.</li>
 * <li>(Optional) A copy of the {@code jdistlib} jar available from <a href=
 * "http://jdistlib.sourceforge.net/">http://jdistlib.sourceforge.net/</a>. This
 * is only required if at least one property specification is probabilistic. For
 * purely combinatorial specifications, {@code jdistlib} is not required. Ensure
 * that the downloaded jar file is on the build path or the class path prior to
 * compilation. For example, the jar file can be placed in
 * {@code $ARTISYNTH_HOME/lib} (for Linux/Mac OSX) or
 * {@code %ARTISYNTH_HOME%\lib} (for Windows). {@code ArtiSynth} does not ship
 * with {@code jdistlib} because of license incompatibilities.</li>
 * </ul>
 * BatchSim can be run both from within {@code Eclipse} and from the
 * command-line. Although running from the command-line may seem daunting at
 * first, doing so is actually easier than running from within {@code Eclipse},
 * because BatchSim can run from the directory in which the input file and
 * optional {@code Jython} script already reside, and each manager and worker
 * gets its own output console.
 * <p>
 * Suppose the worker subclass, the input file, and the {@code Jython} driver
 * script reside in a package called {@code batch}. Then, to run BatchSim from
 * the command-line, do the following:
 * <ol>
 * <li>Make sure all the environment variables needed to run {@code ArtiSynth}
 * are set. In addition, make sure that {@code jdistlib} is on the class path
 * (if running probabilistic simulations), and also that
 * {@code artisynth_core/lib/*}, {@code artisynth_core/classes}, and
 * {@code artisynth_projects/classes} (and optionally
 * {@code artisynth_models/classes}) are on the class path.</li>
 * <li>Make sure that the property specifications input file is named
 * "props.psl", which is the default filename that the manager looks for. If the
 * file is named something else, and additional command-line argument giving the
 * alternative filename must be provided to the manager.</li>
 * <li>In one terminal window, {@code cd} to {@code batch}. Then, run<br>
 * <code>&nbsp;&nbsp;java artisynth.tools.batchsim.BatchManager [options]</code><br>
 * to start a manager as a stand alone application, or run<br>
 * <code>&nbsp;&nbsp;artisynth [other ArtiSynth options] -model &lt;model_classname&gt;<br>
 * &nbsp;&nbsp;&nbsp;&nbsp;-script &lt;path_to_batchsim_package&gt;/managerInit.py &lt;script_args_destined_for_manager&gt;</code><br>
 * (as a single command) to start a manager within an instance of
 * {@code ArtiSynth}. The only benefit to running the manager within an instance
 * of {@code ArtiSynth} is that only then can the manager verify that the
 * property specifications listed in the input file represent valid
 * {@code Properties} and that the values for each {@code Property} are valid
 * for the <b>currently-loaded</b> {@code RootModel} subclass (at the time the
 * manager's constructor is called). Once this has been verified, it is much
 * easier to run the manager as a stand alone application (at least until the
 * next time a change to the input file is made). Note that a worker will also
 * validate the property specifications, but unlike the manager, this validation
 * is done only as quickly as the worker receives property-value pairs from the
 * manager in the form of simulation tasks.</li>
 * <li>Decide how many workers to run. Then, open that many additional terminal
 * windows (one for each worker), and {@code cd} to {@code batch} in each
 * one.</li>
 * <li>In each worker terminal window, run<br>
 * <code>&nbsp;&nbsp;artisynth [other ArtiSynth options] -model &lt;model_classname&gt;</code><br>
 * <code>&nbsp;&nbsp;&nbsp;&nbsp;-script &lt;driver_script_filename&gt; &lt;script_args_destined_for_worker&gt;</code><br>
 * to start a worker.</li>
 * <li>If no errors have been made, BatchSim should now be performing
 * simulations as per the property specifications in the input file. In
 * addition, the simulations will be performed in parallel if more than one
 * worker was started.</li>
 * <p>
 * Running BatchSim from within {@code Eclipse} is done is a similar fashion,
 * except that in {@code Eclipse} the working directory is the project root
 * directory (i.e. {@code artisynth_core}, {@code artisynth_models}, or
 * {@code artisynth_projects}), so the path to the input file and script will
 * have to be specified relative to those directories as opposed to
 * {@code batch}. Running BatchSim from within {@code Eclipse} is made even more
 * difficult because {@code Eclipse} only has one Console, even though BatchSim
 * requires that at least 2 programs (the manager and at least one worker) are
 * running simultaneously. Thus, the Console output from at least one program
 * will not be visible. To specify command-line arguments to a program that is
 * started from {@code Eclipse}, right-click on the program file and select
 * <i>Run As</i> > <i>Run Configurations...</i> from the context menu that
 * appears. In the resulting window, click on the <i>Arguments</i> tab, and type
 * the command-line arguments in the text box labeled "Program Arguments". Then,
 * click <i>Apply</i> and run the program.
 * <p>
 * The preceding examples for running BatchSim (from the command-line or from
 * within {@code Eclipse}) present only the simplest and most common use case.
 * It allows BatchSim to run in its "default" mode. Additional modes and
 * behaviours can be specified by appropriate command-line arguments. See
 * {@link artisynth.tools.batchsim.BatchWorkerBase BatchWorkerBase} and
 * {@link artisynth.tools.batchsim.BatchManager BatchManager} for additional
 * details.
 * <h2>3.2. Syntax highlighting for the input file</h2>
 * The input file containing property specifications is actually code written in
 * a simple programming language called the Property Specification Language (PSL;
 * see Section 2), hence the ".psl" extension. Therefore, syntax highlighting for
 * this language has been developed for Eclipse and Vim. See the "syntaxColoring"
 * subpackage of the "batchsim" package for instructions on how to get this
 * feature up and running.
 * 
 * @author Francois Roewer-Despres
 * @version 1.0
 */
package artisynth.tools.batchsim;