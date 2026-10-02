package it.unive.pylisa.testing;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.FunctionalLattice;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.HeapLocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.OutOfScopeIdentifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.analysis.ValueReader;
import it.unive.pylisa.libraries.natives.AssumptionBranch;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

/**
 * A read-only view of one analysis state at one program point, answering the
 * questions a test asks about it: which variables exist, which heap objects a
 * reference denotes, which value an expression has, which errors may have been
 * raised.
 * <p>
 * Every question is answered through the analysis itself (heap rewriting and
 * type inference of the configured domains), so that references and fields are
 * interpreted exactly as the analysed program interprets them.
 * </p>
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
final class StateView<A extends AbstractLattice<A>, D extends AbstractDomain<A>> {

	private final Analysis<A, D> analysis;

	private final AnalysisState<A> state;

	private final Statement point;

	private final ValueReader reader;

	/**
	 * Builds the view.
	 *
	 * @param analysis the analysis that computed the state
	 * @param state    the state
	 * @param point    the program point the state refers to
	 * @param reader   the reader for the configured value domain
	 */
	StateView(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Statement point,
			ValueReader reader) {
		this.analysis = analysis;
		this.state = state;
		this.point = point;
		this.reader = reader;
	}

	/**
	 * Yields the program point this view refers to.
	 *
	 * @return the statement
	 */
	Statement point() {
		return point;
	}

	/**
	 * Yields whether some execution reaches this point normally, that is,
	 * without an error being raised.
	 *
	 * @return {@code true} if the normal execution state is not bottom
	 */
	boolean isReachable() {
		return !state.getExecution().isBottom() && !state.getExecutionState().isBottom();
	}

	/**
	 * Yields the names of the types of the errors that may have been raised by
	 * the time this point is reached.
	 *
	 * @return the error type names, sorted
	 */
	Set<String> errors() {
		Set<String> names = new TreeSet<>();
		Set<AnalysisState.Error> raised = state.getErrors().getKeys();
		if (raised != null)
			raised.forEach(error -> names.add(error.getType().toString()));
		Set<Type> smashed = state.getSmashedErrors().getKeys();
		if (smashed != null)
			smashed.forEach(type -> names.add(type.toString()));
		return names;
	}

	/**
	 * Yields the errors that may have been raised by the time this point is
	 * reached, each with the call that raised it. Errors the analysis smashed
	 * together by type have no call.
	 *
	 * @return the errors
	 */
	Set<ErrorSite> errorSites() {
		Set<ErrorSite> sites = new HashSet<>();
		Set<AnalysisState.Error> raised = state.getErrors().getKeys();
		if (raised != null)
			for (AnalysisState.Error error : raised) {
				Statement thrower = error.getThrower();
				String call = calleeOf(thrower);
				int line = thrower != null && thrower.getLocation() instanceof SourceCodeLocation source
						? source.getLine()
						: -1;
				sites.add(new ErrorSite(error.getType().toString(), call, line));
			}
		Set<Type> smashed = state.getSmashedErrors().getKeys();
		if (smashed != null)
			smashed.forEach(type -> sites.add(new ErrorSite(type.toString(), null, -1)));
		return sites;
	}

	/**
	 * Yields, for each error of a type raised on a line, the assumptions the
	 * branch that raised it depends on: empty for an error raised whatever
	 * the assumptions.
	 */
	Set<Set<String>> errorMarks(
			String type,
			int line) {
		Set<Set<String>> marks = new HashSet<>();
		Set<AnalysisState.Error> raised = state.getErrors().getKeys();
		if (raised != null)
			for (AnalysisState.Error error : raised) {
				Statement thrower = error.getThrower();
				if (error.getType().toString().equals(type) && thrower != null
						&& thrower.getLocation() instanceof SourceCodeLocation source && source.getLine() == line)
					marks.add(thrower instanceof AssumptionBranch branch ? branch.assumptions() : Set.of());
			}
		return marks;
	}

	/**
	 * Yields whether every raised error has a thrower that is a statement of
	 * its CFG, or an expression whose chain of parent statements ends at one.
	 */
	boolean everyErrorRaisedWithinProgram() {
		Set<AnalysisState.Error> raised = state.getErrors().getKeys();
		if (raised == null)
			return true;
		for (AnalysisState.Error error : raised)
			if (!chainsToProgram(error.getThrower()))
				return false;
		return true;
	}

	private static boolean chainsToProgram(
			Statement thrower) {
		Statement current = thrower;
		while (current instanceof Expression expression && expression.getParentStatement() != null)
			current = expression.getParentStatement();
		Statement root = current;
		// identity, not equality: a synthetic node may equal a program node
		return root != null && root.getCFG() != null
				&& root.getCFG().getNodes().stream().anyMatch(node -> node == root);
	}

	/**
	 * Names what a statement that raised an error calls: the callees of a
	 * call (such as {@code mylib.Widget::update}), or the
	 * construct itself.
	 */
	private static String calleeOf(
			Statement thrower) {
		if (thrower instanceof AssumptionBranch branch)
			return calleeOf(branch.getParentStatement());
		if (thrower instanceof NativeCall call && !call.getTargets().isEmpty())
			return call.getTargets().stream()
					// a Python callable is a unit whose code is its $call member
					.map(target -> "$call".equals(target.getDescriptor().getName())
							? target.getDescriptor().getUnit().getName()
							: target.getDescriptor().getFullName())
					.sorted()
					.collect(Collectors.joining("|"));
		if (thrower instanceof Call call)
			return call.getFullTargetName();
		if (thrower instanceof NaryExpression expression)
			return expression.getConstructName();
		return String.valueOf(thrower);
	}

	/**
	 * Finds the program variable with the given name: a local variable of the
	 * function this point belongs to, or else a global of the {@code __main__}
	 * module, or else the only global of any module with that name.
	 *
	 * @param name the variable name, as written in the Python source
	 *
	 * @return the variable, if it exists in this state
	 */
	Optional<Identifier> variable(
			String name) {
		List<Identifier> inScope = inScopeIdentifiers();
		Optional<Identifier> local = inScope.stream().filter(id -> id.getName().equals(name)).findFirst();
		if (local.isPresent())
			return local;
		Optional<Identifier> main = inScope.stream()
				.filter(id -> id.getName().equals("$__main__::" + name))
				.findFirst();
		if (main.isPresent())
			return main;
		List<Identifier> globals = inScope.stream()
				.filter(id -> id.getName().startsWith("$") && id.getName().endsWith("::" + name))
				.collect(Collectors.toList());
		return globals.size() == 1 ? Optional.of(globals.get(0)) : Optional.empty();
	}

	/**
	 * Yields the heap locations of the objects a reference may point to.
	 *
	 * @param reference an expression denoting a reference
	 *
	 * @return the heap locations
	 */
	Set<HeapLocation> objectsPointedBy(
			SymbolicExpression reference) {
		return rewrite(new HeapDereference(Untyped.INSTANCE, reference, PySyntheticLocation.INSTANCE)).stream()
				.filter(HeapLocation.class::isInstance)
				.map(HeapLocation.class::cast)
				.collect(Collectors.toCollection(LinkedHashSet::new));
	}

	/**
	 * Yields the abstract objects of the given type that exist in this state,
	 * each with a reference that points to it.
	 *
	 * @param typeName the name of the type, such as
	 *                     {@code mylib.Widget}
	 *
	 * @return the references to the objects, by heap location
	 */
	Map<HeapLocation, SymbolicExpression> objectsOfType(
			String typeName) {
		Map<HeapLocation, SymbolicExpression> objects = new LinkedHashMap<>();
		for (Identifier id : inScopeIdentifiers())
			if (id instanceof AllocationSite site && site.getField() == null)
				for (Type type : typesOf(site))
					if (type.toString().equals(typeName))
						objects.put(site,
								new HeapReference(new ReferenceType(type), site, PySyntheticLocation.INSTANCE));
		return objects;
	}

	/**
	 * Yields a reference to one object, which denotes that object only.
	 *
	 * @param location the heap location of the object
	 *
	 * @return the reference
	 */
	SymbolicExpression referenceTo(
			HeapLocation location) {
		return new HeapReference(new ReferenceType(Untyped.INSTANCE), location, PySyntheticLocation.INSTANCE);
	}

	/**
	 * Yields the value of an expression, joined over everything the expression
	 * may denote.
	 *
	 * @param expression the expression
	 *
	 * @return the value, or empty if no execution gives the expression a value
	 */
	Optional<Val> valueOf(
			SymbolicExpression expression) {
		try {
			return reader.valueOf(analysis, state, expression, point);
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot read " + expression + " at " + point.getLocation(), e);
		}
	}

	/**
	 * Yields the runtime types an expression may have.
	 *
	 * @param expression the expression
	 *
	 * @return the types
	 */
	Set<Type> typesOf(
			SymbolicExpression expression) {
		try {
			return analysis.getRuntimeTypesOf(state, expression, point);
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot type " + expression + " at " + point.getLocation(), e);
		}
	}

	/**
	 * Yields whether the state stores something for an expression: it denotes
	 * at least one identifier, and the value or the type component holds every
	 * identifier it denotes. A field never assigned is not stored, while its
	 * value and its types read as unknown.
	 *
	 * @param expression the expression
	 *
	 * @return {@code true} if the state stores the expression
	 */
	boolean isStored(
			SymbolicExpression expression) {
		SimpleAbstractState<?, ?, ?> components = stateComponents();
		Set<SymbolicExpression> denoted = rewrite(expression);
		return !denoted.isEmpty() && denoted.stream()
				.allMatch(denotation -> denotation instanceof Identifier identifier
						&& (components.valueState.knowsIdentifier(identifier)
								|| components.typeState.knowsIdentifier(identifier)));
	}

	/**
	 * Builds the expression that accesses a field of the object a reference
	 * points to, as the Python attribute access {@code reference.name} does.
	 *
	 * @param reference an expression denoting a reference
	 * @param name      the field name
	 *
	 * @return the field access
	 */
	static SymbolicExpression field(
			SymbolicExpression reference,
			String name) {
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, reference, PySyntheticLocation.INSTANCE);
		Variable child = new Variable(Untyped.INSTANCE, name, PySyntheticLocation.INSTANCE);
		return new AccessChild(Untyped.INSTANCE, container, child, PySyntheticLocation.INSTANCE);
	}

	private Set<SymbolicExpression> rewrite(
			SymbolicExpression expression) {
		try {
			Set<SymbolicExpression> rewritten = new LinkedHashSet<>();
			analysis.rewrite(state, expression, point).forEach(rewritten::add);
			return rewritten;
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot rewrite " + expression + " at " + point.getLocation(), e);
		}
	}

	private List<Identifier> inScopeIdentifiers() {
		if (!(stateComponents().typeState instanceof FunctionalLattice<?, ?, ?> types) || types.getKeys() == null)
			return List.of();
		return types.getKeys().stream()
				.filter(Identifier.class::isInstance)
				.map(Identifier.class::cast)
				.filter(id -> !(id instanceof OutOfScopeIdentifier))
				.collect(Collectors.toList());
	}

	private SimpleAbstractState<?, ?, ?> stateComponents() {
		if (state.getExecutionState() instanceof SimpleAbstractState<?, ?, ?> components)
			return components;
		throw new IllegalStateException("Only states made of heap, value and type components are supported, got "
				+ state.getExecutionState().getClass().getName());
	}
}
