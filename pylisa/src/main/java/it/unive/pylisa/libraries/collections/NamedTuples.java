package it.unive.pylisa.libraries.collections;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.NamedTupleClassType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collection;
import java.util.List;

/**
 * The tuples created by the classes of {@code collections.namedtuple} (see
 * {@link NamedTupleClassType}): tuples of the class {@code NamedTuple} (a
 * subclass of {@code Tuple}), whose elements are also their attributes, named
 * by the fields of the class.
 */
public final class NamedTuples {

	private NamedTuples() {
	}

	/**
	 * Whether the values of the given type are tuples created by the classes of
	 * {@code collections.namedtuple}.
	 *
	 * @param t the type
	 *
	 * @return whether the values are such tuples
	 */
	public static boolean isNamedTuple(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.NAMED_TUPLE);
	}

	/**
	 * Calls the given class with the given arguments: a new tuple whose
	 * elements are the arguments, matched to the fields of the class by
	 * position, and then by name (for the keyword arguments). {@code TypeError}
	 * is raised if a field has no argument, or has more than one, or if an
	 * argument matches no field. With a starred argument (e.g. {@code *xs}),
	 * the elements of the tuple are not known, and {@code TypeError} might be
	 * raised.
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param state    the state where the arguments have been evaluated
	 * @param cls      the class
	 * @param actuals  the expressions of the arguments
	 * @param args     the values of the arguments
	 * @param pp       the statement calling the class
	 * @param metas    where the receiver of the tuple is added
	 *
	 * @return the state after the call
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			NamedTupleClassType cls,
			Expression[] actuals,
			ExpressionSet[] args,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = new DerivedLocation(pp.getLocation(), "namedtuple");
		List<String> fields = cls.getFields();
		ExpressionSet[] values = new ExpressionSet[fields.size()];
		boolean typeError = false, starred = false;
		int position = 0;
		for (int i = 0; i < actuals.length; i++)
			if (actuals[i] instanceof StarExpression)
				starred = true;
			else if (actuals[i] instanceof NamedParameterExpression) {
				int index = fields.indexOf(((NamedParameterExpression) actuals[i]).getParameterName());
				if (index < 0 || values[index] != null)
					// no such field, or a field with two arguments
					typeError = true;
				else
					values[index] = args[i];
			} else if (position < values.length)
				values[position++] = args[i];
			else
				// too many arguments
				typeError = true;

		AnalysisState<A> result = state.bottomExecution();
		if (starred) {
			for (int i = 0; i < values.length; i++)
				values[i] = new ExpressionSet(new PushAny(Untyped.INSTANCE, loc));
			typeError = true;
		}
		for (ExpressionSet value : values)
			typeError |= value == null;
		if (starred || !typeError) {
			Type type = PyClassType.lookup(LibrarySpecificationProvider.NAMED_TUPLE);
			AnalysisState<A> created = TupleCreation.create(analysis, state, pp, loc, type, values, metas);
			for (SymbolicExpression tuple : created.getExecutionExpressions()) {
				// the elements are also the attributes
				AnalysisState<A> current = created;
				HeapDereference deref = new HeapDereference(type, tuple, loc);
				for (int i = 0; i < values.length; i++) {
					AccessChild attribute = new AccessChild(Untyped.INSTANCE, deref,
							new Variable(Untyped.INSTANCE, fields.get(i), loc), loc);
					AnalysisState<A> next = current.bottomExecution();
					for (SymbolicExpression value : values[i])
						next = next.lub(analysis.assign(current, attribute, value, pp));
					current = next;
				}
				result = result.lub(analysis.smallStepSemantics(current, tuple, pp));
			}
		}
		if (typeError)
			result = result.lub(PyExceptions.raise(analysis, state, pp.getCFG(), pp.getLocation(), pp,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
