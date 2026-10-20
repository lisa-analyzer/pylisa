package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * The unpacking of a value into a sequence of targets, as in
 * {@code a, *b, c = value} or {@code for k, v in pairs}. If the value is a list
 * or a tuple whose length is known, its elements are assigned to the targets
 * (the starred target, if any, receives a new list with the elements that are
 * not assigned to the other targets), and {@code ValueError} is raised if the
 * number of elements does not match the number of targets. Otherwise, each
 * target is assigned any of the elements (the starred one a list of any of
 * them), and {@code ValueError} might be raised. Values that cannot be iterated
 * (e.g. an {@code int}) raise {@code TypeError}.
 */
public final class Unpacking {

	private Unpacking() {
	}

	/**
	 * The assignment of a value to a single target.
	 *
	 * @param <A> the kind of the analysis state
	 */
	@FunctionalInterface
	public interface Assigner<A extends AbstractLattice<A>> {

		/**
		 * Assigns the value to the target.
		 *
		 * @param state  the current state
		 * @param target the target
		 * @param value  the value
		 *
		 * @return the state after the assignment
		 *
		 * @throws SemanticException if the analysis fails
		 */
		AnalysisState<A> assign(
				AnalysisState<A> state,
				Expression target,
				SymbolicExpression value)
				throws SemanticException;
	}

	/**
	 * Unpacks the value into the targets.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the value has been computed
	 * @param value           the value
	 * @param targets         the targets
	 * @param star            the index of the starred target (a
	 *                            {@link StarExpression}), or {@code -1}
	 * @param pp              the statement unpacking the value
	 * @param assigner        the assignment of the elements to the targets
	 *
	 * @return the state after the unpacking
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unpack(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression value,
			Expression[] targets,
			int star,
			Statement pp,
			Assigner<A> assigner)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = pp.getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, pp);
		AnalysisState<A> result = state.bottomExecution();
		boolean valueError = false, typeError = false;

		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp)) {
			valueError = true;
			result = result.lub(any(interprocedural, state,
					analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp), targets, star, pp,
					assigner));
		} else
			for (Type t : types)
				if (ListModel.isList(t) || isTuple(t)) {
					ListModel.Shape shape = ListModel.shape(analysis, state, value, pp);
					HeapDereference deref = new HeapDereference(t.asPointerType().getInnerType(), value, loc);
					Integer n = shape.length();
					if (n == null) {
						valueError = true;
						AnalysisState<A> elements = state.bottomExecution();
						if (shape.elements() == null)
							elements = analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
						else
							for (SymbolicExpression field : shape.elements())
								elements = elements.lub(analysis.smallStepSemantics(state,
										new AccessChild(Untyped.INSTANCE, deref, field, loc), pp));
						result = result.lub(any(interprocedural, state, elements, targets, star, pp, assigner));
					} else if (star < 0 ? n != targets.length : n < targets.length - 1)
						// too many or too few values
						valueError = true;
					else
						result = result.lub(exact(interprocedural, state, deref, n, targets, star, pp, assigner));
				} else if (DictModel.isDict(t)) {
					valueError = true;
					result = result.lub(any(interprocedural, state, DictModel.keys(analysis, state, value, pp),
							targets, star, pp, assigner));
				} else if (t.isStringType() || t instanceof PyBytesType) {
					valueError = true;
					Type element = t.isStringType() ? StringType.INSTANCE : Int32Type.INSTANCE;
					result = result.lub(any(interprocedural, state,
							analysis.smallStepSemantics(state, new PushAny(element, loc), pp), targets, star, pp,
							assigner));
				} else if (t.isCodeMemberType()
						|| (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t)))
					// e.g. an int, or a function
					typeError = true;
				else {
					// e.g. an object whose class defines __iter__
					valueError = true;
					result = result.lub(any(interprocedural, state,
							analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp), targets, star,
							pp, assigner));
				}

		if (valueError)
			result = result.lub(PyExceptions.raise(analysis, state, pp.getCFG(), loc, pp,
					LibrarySpecificationProvider.VALUE_ERROR));
		if (typeError)
			result = result.lub(PyExceptions.raise(analysis, state, pp.getCFG(), loc, pp,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	// whether the values of type t are tuples
	static boolean isTuple(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.TUPLE);
	}

	// the elements of a sequence of known length n, matching the targets
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> exact(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			HeapDereference deref,
			int n,
			Expression[] targets,
			int star,
			Statement pp,
			Assigner<A> assigner)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> current = state;
		int after = star < 0 ? 0 : targets.length - star - 1;
		for (int i = 0; i < targets.length; i++) {
			if (i == star)
				continue;
			// the targets after the starred one take the last elements
			int index = star >= 0 && i > star ? n - after + (i - star - 1) : i;
			ExpressionSet elements = element(analysis, state, deref, index, pp);
			AnalysisState<A> next = current.bottomExecution();
			for (SymbolicExpression element : elements)
				next = next.lub(assigner.assign(current, targets[i], element));
			current = next;
		}
		if (star >= 0) {
			// the starred target takes the elements in between, in a list
			int count = n - targets.length + 1;
			ExpressionSet[] middle = new ExpressionSet[count];
			for (int i = 0; i < count; i++)
				middle[i] = element(analysis, state, deref, star + i, pp);
			ListCreation creation = new ListCreation(pp.getCFG(), new DerivedLocation(loc, "star"));
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, current, middle,
					new StatementStore<>(current));
			current = assignStarred(created, targets[star], creation, assigner);
		}
		return current;
	}

	// any of the elements to each target
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> any(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			AnalysisState<A> elements,
			Expression[] targets,
			int star,
			Statement pp,
			Assigner<A> assigner)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = pp.getLocation();
		if (elements.getExecutionExpressions().isEmpty() && !elements.getExecutionState().isBottom())
			// no element is known
			elements = analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		AnalysisState<A> current = state.lub(elements.bottomExecution());
		for (int i = 0; i < targets.length; i++) {
			if (i == star)
				continue;
			AnalysisState<A> next = current.bottomExecution();
			for (SymbolicExpression element : elements.getExecutionExpressions())
				next = next.lub(assigner.assign(current, targets[i], element));
			current = next;
		}
		if (star >= 0) {
			// the starred target takes any of the elements, in a list
			ListCreation creation = new ListCreation(pp.getCFG(), new DerivedLocation(loc, "star"));
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, current, new ExpressionSet[0],
					new StatementStore<>(current));
			Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
			AnalysisState<A> filled = created;
			for (SymbolicExpression list : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				for (SymbolicExpression element : elements.getExecutionExpressions())
					// the list might also be empty
					filled = filled.lub(analysis.assign(created, unknown, element, pp));
			}
			current = assignStarred(filled.withExecutionExpressions(created.getExecutionExpressions()),
					targets[star], creation, assigner);
		}
		return current;
	}

	// assigns the list built for the starred target
	private static <A extends AbstractLattice<A>> AnalysisState<A> assignStarred(
			AnalysisState<A> created,
			Expression starred,
			ListCreation creation,
			Assigner<A> assigner)
			throws SemanticException {
		Expression target = ((StarExpression) starred).getSubExpression();
		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression list : created.getExecutionExpressions())
			result = result.lub(assigner.assign(created, target, list));
		return result.forgetIdentifiers(creation.getMetaVariables(), creation);
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ExpressionSet element(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			int index,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		return analysis.smallStepSemantics(state,
				new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, index, loc), loc), pp)
				.getExecutionExpressions();
	}
}
