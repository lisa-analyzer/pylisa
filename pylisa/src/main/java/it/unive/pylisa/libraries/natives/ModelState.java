package it.unive.pylisa.libraries.natives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.HeapLocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyExceptionType;
import it.unive.pylisa.program.PyProgram;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * One analysis state inside the model of a library call, together with the
 * operations that library models perform on it: allocating objects, reading
 * and writing their fields, splitting on conditions, raising exceptions and
 * producing the result of the call.
 * <p>
 * Instances are immutable: every operation yields a new state and leaves this
 * one unchanged, so that a model can explore alternative outcomes (a valid and
 * an invalid argument, say) from the same starting point and join them.
 * </p>
 * <p>
 * After an operation that computes values ({@link #allocate}, {@link #read},
 * {@link #returning}), those values are available through {@link #values()}.
 * </p>
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
public final class ModelState<A extends AbstractLattice<A>, D extends AbstractDomain<A>> {

	/**
	 * A step of a model that may fail with a semantic exception.
	 *
	 * @param <A> the kind of abstract state
	 * @param <D> the kind of abstract domain
	 * @param <T> the kind of input of the step
	 */
	@FunctionalInterface
	public interface Step<A extends AbstractLattice<A>, D extends AbstractDomain<A>, T> {

		/**
		 * Performs the step.
		 *
		 * @param state the state the step starts from
		 * @param input the input of the step
		 *
		 * @return the state after the step
		 *
		 * @throws SemanticException if the step cannot be performed
		 */
		ModelState<A, D> apply(
				ModelState<A, D> state,
				T input)
				throws SemanticException;
	}

	private final Analysis<A, D> analysis;

	private final AnalysisState<A> state;

	private final ProgramPoint point;

	private final Statement call;

	/**
	 * The assumptions of the environment that the executions of this state
	 * need not satisfy: they exist because each of them may not hold.
	 */
	private final SortedSet<String> marks;

	/**
	 * Builds the state.
	 *
	 * @param analysis the analysis computing the state
	 * @param state    the analysis state
	 * @param point    the program point of the model
	 * @param call     the call being modelled, which is the statement that
	 *                     raises the exceptions of the model
	 */
	public ModelState(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ProgramPoint point,
			Statement call) {
		this(analysis, state, point, call, Collections.emptySortedSet());
	}

	private ModelState(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ProgramPoint point,
			Statement call,
			SortedSet<String> marks) {
		this.analysis = analysis;
		this.state = state;
		this.point = point;
		this.call = call;
		this.marks = marks;
	}

	/**
	 * Yields this state as the executions that exist because an assumption of
	 * the environment may not hold: the errors raised from it, in this model
	 * call, are recorded as depending on that assumption.
	 *
	 * @param assumption the name of the assumption
	 *
	 * @return the state
	 */
	public ModelState<A, D> assuming(
			String assumption) {
		SortedSet<String> more = new TreeSet<>(marks);
		more.add(assumption);
		return new ModelState<>(analysis, state, point, call, Collections.unmodifiableSortedSet(more));
	}

	/**
	 * Sets a carried mark (see {@link CarriedMarks}) on the executions of this
	 * state: the model calls that follow, on executions where it is set, start
	 * with it.
	 *
	 * @param name the name of the mark, one of the program's carried marks
	 *
	 * @return the state with the mark set
	 *
	 * @throws SemanticException if the mark cannot be set
	 */
	public ModelState<A, D> carry(
			String name)
			throws SemanticException {
		// a flag the program's entry does not unset could be read as set on a
		// join with executions that never set it
		if (!(call.getProgram() instanceof PyProgram)
				|| !setting(CarriedMarks.class).orElse(CarriedMarks.NONE).names().contains(name))
			throw new SemanticException("The model of " + call + " at " + call.getLocation() + " carries the mark "
					+ name + ", which the program does not carry");
		return assign(CarriedMarks.flag(name, call.getLocation()), new Expressions(call.getLocation()).bool(true));
	}

	/**
	 * Yields this state with the carried marks (see {@link CarriedMarks}) that
	 * are set on every one of its executions.
	 *
	 * @return the state with those marks
	 *
	 * @throws SemanticException if a mark cannot be read
	 */
	ModelState<A, D> withCarriedMarks()
			throws SemanticException {
		ModelState<A, D> result = this;
		Expressions build = new Expressions(call.getLocation());
		// a program that pylisa did not translate carries no settings
		if (!(call.getProgram() instanceof PyProgram))
			return result;
		for (String name : setting(CarriedMarks.class).orElse(CarriedMarks.NONE).names())
			if (satisfies(build.equal(CarriedMarks.flag(name, call.getLocation()), build.bool(true)))
					== Satisfiability.SATISFIED)
				result = result.assuming(name);
		return result;
	}

	/**
	 * Yields the underlying analysis state.
	 *
	 * @return the analysis state
	 */
	public AnalysisState<A> analysisState() {
		return state;
	}

	/**
	 * Yields the values computed by the last value-producing operation.
	 *
	 * @return the values
	 */
	public ExpressionSet values() {
		return state.getExecutionExpressions();
	}

	/**
	 * Yields whether no execution continues normally from this state.
	 *
	 * @return {@code true} if the normal execution state is bottom
	 */
	public boolean isUnreachable() {
		return state.getExecution().isBottom() || state.getExecutionState().isBottom();
	}

	/**
	 * Yields a setting of the environment the analysed program runs in, as
	 * the client of the analysis gave it with the program.
	 *
	 * @param <T>  the type of the setting
	 * @param type the type the setting was registered under
	 *
	 * @return the setting, or empty if the client gave none of that type
	 */
	public <T> Optional<T> setting(
			Class<T> type) {
		return PyProgram.setting(call, type);
	}

	/**
	 * Yields the state that describes no execution continuing normally, but
	 * keeps the errors raised so far.
	 *
	 * @return the state
	 */
	public ModelState<A, D> unreachable() {
		return with(state.bottomExecution());
	}

	/**
	 * Joins this state with another one: the result describes the executions
	 * of both.
	 *
	 * @param other the other state
	 *
	 * @return the join
	 *
	 * @throws SemanticException if the states cannot be joined
	 */
	public ModelState<A, D> lub(
			ModelState<A, D> other)
			throws SemanticException {
		// the join depends only on the assumptions both parts depend on; a
		// part with no execution continuing adds no execution to depend on
		SortedSet<String> common;
		if (isUnreachable())
			// when both parts are unreachable the marks are those of the other
			// part: harmless, since nothing is raised from an unreachable state
			common = other.marks;
		else if (other.isUnreachable())
			common = marks;
		else {
			SortedSet<String> both = new TreeSet<>(marks);
			both.retainAll(other.marks);
			common = Collections.unmodifiableSortedSet(both);
		}
		return new ModelState<>(analysis, state.lub(other.state), point, call, common);
	}

	/**
	 * Applies a step to every given value, starting each time from this state,
	 * and joins the outcomes. This is how a model handles an argument that may
	 * denote several values.
	 *
	 * @param <T>    the kind of values
	 * @param values the values
	 * @param step   the step
	 *
	 * @return the join of the outcomes, unreachable if this state is
	 *             unreachable
	 *
	 * @throws SemanticException if a step fails, or if there are no values
	 *                               although an execution reaches this state
	 */
	public <T> ModelState<A, D> forEach(
			Iterable<T> values,
			Step<A, D, T> step)
			throws SemanticException {
		ModelState<A, D> result = unreachable();
		boolean any = false;
		for (T value : values) {
			any = true;
			result = result.lub(step.apply(this, value));
		}
		if (!any && !isUnreachable())
			// an execution reaches this point, so it must continue somehow
			throw new SemanticException("A model of " + call + " at " + call.getLocation()
					+ " iterates over no values in a reachable state");
		return result;
	}

	/**
	 * Applies a step to every combination of values of the given arguments,
	 * starting each time from this state, and joins the outcomes. This is how
	 * a model handles arguments that may each denote several values.
	 *
	 * @param arguments the possible values of each argument
	 * @param step      the step, receiving one value per argument, in order
	 *
	 * @return the join of the outcomes, unreachable if this state is
	 *             unreachable
	 *
	 * @throws SemanticException if a step fails, or if an argument has no values
	 *                               although an execution reaches this state
	 */
	public ModelState<A, D> forEachCombination(
			List<ExpressionSet> arguments,
			Step<A, D, List<SymbolicExpression>> step)
			throws SemanticException {
		return combine(arguments, 0, new ArrayList<>(), step);
	}

	private ModelState<A, D> combine(
			List<ExpressionSet> arguments,
			int next,
			List<SymbolicExpression> chosen,
			Step<A, D, List<SymbolicExpression>> step)
			throws SemanticException {
		if (next == arguments.size())
			return step.apply(this, List.copyOf(chosen));
		return forEach(arguments.get(next), (current, value) -> {
			chosen.add(value);
			ModelState<A, D> outcome = current.combine(arguments, next + 1, chosen, step);
			chosen.remove(chosen.size() - 1);
			return outcome;
		});
	}

	/**
	 * Allocates a new object. Its references become the computed values.
	 *
	 * @param type the type of the object
	 * @param site the allocation site: objects allocated at the same site are
	 *                 represented by the same abstract object
	 *
	 * @return the state after the allocation
	 *
	 * @throws SemanticException if the allocation cannot be performed
	 */
	public ModelState<A, D> allocate(
			Type type,
			CodeLocation site)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		MemoryAllocation allocation = new MemoryAllocation(type, site, false);
		// the allocation is resolved to its abstract site before it happens:
		// resolving it again afterwards would find the site already allocated
		// and yield its weak (summary) version, so the new object would not be
		// updated strongly
		ExpressionSet sites = analysis.rewrite(state, allocation, point);
		AnalysisState<A> allocated = analysis.smallStepSemantics(state, allocation, point);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression location : sites) {
			HeapReference reference = new HeapReference(new ReferenceType(type), location, site);
			result = result.lub(analysis.smallStepSemantics(allocated, reference, point));
		}
		return with(result);
	}

	/**
	 * Reads a field of the object a reference points to. The locations of the
	 * field become the computed values; the values stored there are read by
	 * using those locations as expressions.
	 *
	 * @param reference the reference
	 * @param name      the field name
	 *
	 * @return the state after the read
	 *
	 * @throws SemanticException if the read cannot be performed
	 */
	public ModelState<A, D> read(
			SymbolicExpression reference,
			String name)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		AnalysisState<A> located = analysis.smallStepSemantics(state, field(reference, name), point);
		if (located.getExecutionExpressions().isEmpty())
			throw new SemanticException("A model of " + call + " at " + call.getLocation() + " reads field " + name
					+ " and finds no values in a reachable state");
		boolean placeholder = false;
		for (SymbolicExpression location : located.getExecutionExpressions())
			placeholder |= isPlaceholder(location);
		if (!placeholder)
			return with(located);
		// a field of an object the heap does not track holds an unknown value,
		// whatever was written through the placeholder that stands for it
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression location : located.getExecutionExpressions())
			result = result.lub(analysis.smallStepSemantics(located,
					isPlaceholder(location) ? new PushAny(Untyped.INSTANCE, call.getLocation()) : location, point));
		return with(result);
	}

	/**
	 * Writes a value into a field of the object a reference points to.
	 *
	 * @param reference the reference
	 * @param name      the field name
	 * @param value     the value
	 *
	 * @return the state after the write
	 *
	 * @throws SemanticException if the write cannot be performed
	 */
	public ModelState<A, D> write(
			SymbolicExpression reference,
			String name,
			SymbolicExpression value)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		AnalysisState<A> located = analysis.smallStepSemantics(state, field(reference, name), point);
		// a value read from a field (such as the receiver of
		// self.client.call_async(...)) is first resolved to the location of
		// that field, so that the heap copies what the location points to
		ExpressionSet values = value instanceof AccessChild
				? analysis.rewrite(state, value, point)
				: new ExpressionSet(value);
		List<SymbolicExpression> targets = new ArrayList<>();
		for (SymbolicExpression target : located.getExecutionExpressions())
			if (!isPlaceholder(target))
				targets.add(target);
		if (targets.isEmpty())
			// the reference points to no object the analysis tracks, whose
			// fields are therefore not tracked either
			return this;
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression target : targets)
			for (SymbolicExpression stored : values)
				result = result.lub(analysis.assign(located, target, stored, point));
		return with(result);
	}

	/**
	 * Assigns a value to a variable.
	 *
	 * @param variable the variable
	 * @param value    the value
	 *
	 * @return the state after the assignment
	 *
	 * @throws SemanticException if the assignment cannot be performed
	 */
	public ModelState<A, D> assign(
			Identifier variable,
			SymbolicExpression value)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		return with(analysis.assign(state, variable, value, point));
	}

	/**
	 * Keeps only the executions in which a condition may hold.
	 *
	 * @param condition the condition
	 *
	 * @return the refined state, unreachable if the condition cannot hold
	 *
	 * @throws SemanticException if the condition cannot be evaluated
	 */
	public ModelState<A, D> assume(
			SymbolicExpression condition)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		return with(analysis.assume(state, condition, point, point));
	}

	/**
	 * Keeps only the executions in which a condition may not hold.
	 *
	 * @param condition the condition
	 *
	 * @return the refined state, unreachable if the condition must hold
	 *
	 * @throws SemanticException if the condition cannot be evaluated
	 */
	public ModelState<A, D> assumeNot(
			SymbolicExpression condition)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		return assume(new UnaryExpression(BoolType.INSTANCE, condition, LogicalNegation.INSTANCE,
				call.getLocation()));
	}

	/**
	 * Splits the executions of this state on a condition, continues each part
	 * with its own step, and joins the outcomes. A part that no execution can
	 * take (because the condition certainly holds, or certainly does not) is
	 * not explored. Where the condition is undecided both parts continue from
	 * this state, which the condition does not refine.
	 *
	 * @param condition the condition
	 * @param whenTrue  the step for the executions where the condition holds
	 * @param whenFalse the step for the executions where it does not
	 *
	 * @return the join of the outcomes
	 *
	 * @throws SemanticException if the condition cannot be evaluated or a step
	 *                               fails
	 */
	public ModelState<A, D> branch(
			SymbolicExpression condition,
			Step<A, D, SymbolicExpression> whenTrue,
			Step<A, D, SymbolicExpression> whenFalse)
			throws SemanticException {
		// the parts are not refined by the condition: the value domains may
		// refine summary locations (such as a field of an object allocated in
		// a loop) as if they held a single value, which would wrongly carry
		// the condition past the model
		Satisfiability decided = satisfies(condition);
		ModelState<A, D> result = unreachable();
		if (decided != Satisfiability.NOT_SATISFIED && !isUnreachable())
			result = result.lub(whenTrue.apply(this, condition));
		if (decided != Satisfiability.SATISFIED && !isUnreachable())
			result = result.lub(whenFalse.apply(this, condition));
		return result;
	}

	/**
	 * Splits the executions of this state on whether a value is
	 * {@code None}. The types the value may have decide first, since the
	 * value domains may not track references; where they do not decide, the
	 * comparison with {@code None} does.
	 *
	 * @param value     the value
	 * @param whenNone  the step for the executions where it is {@code None}
	 * @param otherwise the step for the executions where it is not
	 *
	 * @return the join of the outcomes
	 *
	 * @throws SemanticException if the value cannot be evaluated or a step
	 *                               fails
	 */
	public ModelState<A, D> ifNone(
			SymbolicExpression value,
			Step<A, D, SymbolicExpression> whenNone,
			Step<A, D, SymbolicExpression> otherwise)
			throws SemanticException {
		Set<Type> types = runtimeTypes(value);
		if (!types.isEmpty() && types.stream().allMatch(Type::isNullType))
			return whenNone.apply(this, value);
		// only types that certainly exclude None decide: references and
		// plain values, not unknown or missing type information
		if (!types.isEmpty() && types.stream().allMatch(t -> t.isPointerType() || t.isStringType()
				|| t.isNumericType() || t.isBooleanType()))
			return otherwise.apply(this, value);
		// the value domains do not track references: comparing one that may
		// be an object with None decides nothing
		if (types.stream().anyMatch(Type::isPointerType))
			return whenNone.apply(this, value).lub(otherwise.apply(this, value));
		CodeLocation location = call.getLocation();
		return branch(new BinaryExpression(BoolType.INSTANCE, value, new PyNoneConstant(location),
				ComparisonEq.INSTANCE, location), whenNone, otherwise);
	}

	/**
	 * Decides a condition in this state.
	 *
	 * @param condition the condition
	 *
	 * @return whether the condition holds
	 *
	 * @throws SemanticException if the condition cannot be evaluated
	 */
	public Satisfiability satisfies(
			SymbolicExpression condition)
			throws SemanticException {
		return analysis.satisfies(state, condition, point);
	}

	/**
	 * Yields the abstract objects a reference may point to, by the name of
	 * their heap location. A reference that points to no object (such as
	 * {@code None}) yields no name.
	 *
	 * @param reference the reference
	 *
	 * @return the names of the heap locations
	 *
	 * @throws SemanticException if the reference cannot be resolved
	 */
	public Set<String> objects(
			SymbolicExpression reference)
			throws SemanticException {
		Set<String> names = new HashSet<>();
		for (HeapLocation location : locations(reference))
			if (!isPlaceholder(location))
				names.add(location.getName());
		return names;
	}

	/**
	 * Decides whether two references point to the same object. They do when
	 * each points to one and the same abstract object that stands for a
	 * single concrete object; they do not when the objects they may point to
	 * are disjoint; otherwise nothing is decided.
	 *
	 * @param first  the first reference
	 * @param second the second reference
	 *
	 * @return whether they point to the same object
	 *
	 * @throws SemanticException if the references cannot be resolved
	 */
	public Satisfiability sameObject(
			SymbolicExpression first,
			SymbolicExpression second)
			throws SemanticException {
		Set<HeapLocation> left = locations(first);
		Set<HeapLocation> right = locations(second);
		// an object the heap does not track may be any object
		if (left.isEmpty() || right.isEmpty() || left.stream().anyMatch(ModelState::isPlaceholder)
				|| right.stream().anyMatch(ModelState::isPlaceholder))
			return Satisfiability.UNKNOWN;
		Set<String> leftNames = new HashSet<>();
		left.forEach(location -> leftNames.add(location.getName()));
		Set<String> rightNames = new HashSet<>();
		right.forEach(location -> rightNames.add(location.getName()));
		if (leftNames.stream().noneMatch(rightNames::contains))
			return Satisfiability.NOT_SATISFIED;
		if (left.size() == 1 && right.size() == 1 && leftNames.equals(rightNames)
				&& !left.iterator().next().isWeak() && !right.iterator().next().isWeak())
			return Satisfiability.SATISFIED;
		return Satisfiability.UNKNOWN;
	}

	/**
	 * Yields whether an expression is a placeholder the heap uses for objects
	 * it does not track (the object a reference of unknown origin points to):
	 * such a location may stand for any object, and its fields for any value.
	 *
	 * @param expression the expression
	 *
	 * @return {@code true} if it is a placeholder
	 */
	public static boolean isPlaceholder(
			SymbolicExpression expression) {
		if (!(expression instanceof AllocationSite site))
			return false;
		String name = site.getLocationName();
		return name.startsWith("unknown@") || name.startsWith("$pyheap@");
	}

	private Set<HeapLocation> locations(
			SymbolicExpression reference)
			throws SemanticException {
		Set<HeapLocation> locations = new HashSet<>();
		HeapDereference target = new HeapDereference(Untyped.INSTANCE, reference, call.getLocation());
		for (SymbolicExpression location : analysis.rewrite(state, target, point))
			if (location instanceof HeapLocation)
				locations.add((HeapLocation) location);
		return locations;
	}

	/**
	 * Yields the types the value of an expression may have at run time.
	 *
	 * @param expression the expression
	 *
	 * @return the types; empty when nothing is known about them
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public Set<Type> runtimeTypes(
			SymbolicExpression expression)
			throws SemanticException {
		return analysis.getRuntimeTypesOf(state, expression, point);
	}

	/**
	 * Raises an exception: every execution of this state stops normally and
	 * continues as an error of the given type, raised by the modelled call.
	 *
	 * @param type the type of the exception
	 *
	 * @return the state after the raise
	 *
	 * @throws SemanticException if the error cannot be recorded
	 */
	public ModelState<A, D> raise(
			PyExceptionType type)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here, so none raises
			return this;
		// the values computed so far are not the value of anything once the
		// call raises
		AnalysisState<A> cleared = analysis.smallStepSemantics(state, new Skip(call.getLocation()), point);
		// an error of a branch that depends on assumptions is kept apart from
		// the errors the call raises whatever they are
		Statement thrower = marks.isEmpty() ? call : new AssumptionBranch(call, marks);
		return with(analysis.moveExecutionToError(cleared, new AnalysisState.Error(type, thrower), point));
	}

	/**
	 * Makes a value the computed value, that is, the result of the call.
	 *
	 * @param value the value
	 *
	 * @return the state after the evaluation of the value
	 *
	 * @throws SemanticException if the value cannot be evaluated
	 */
	public ModelState<A, D> returning(
			SymbolicExpression value)
			throws SemanticException {
		if (isUnreachable())
			// no execution gets here: nothing happens
			return unreachable();
		return with(analysis.smallStepSemantics(state, value, point));
	}

	/**
	 * Builds the expression that accesses a field of the object a reference
	 * points to.
	 *
	 * @param reference the reference
	 * @param name      the field name
	 *
	 * @return the field access
	 */
	public SymbolicExpression field(
			SymbolicExpression reference,
			String name) {
		CodeLocation location = call.getLocation();
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, reference, location);
		return new AccessChild(Untyped.INSTANCE, container, new Variable(Untyped.INSTANCE, name, location), location);
	}

	private ModelState<A, D> with(
			AnalysisState<A> next) {
		return new ModelState<>(analysis, next, point, call, marks);
	}
}
