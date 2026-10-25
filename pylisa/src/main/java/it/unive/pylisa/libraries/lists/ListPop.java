package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.lists.ListModel.Shape;

/**
 * Native implementation of {@code list.pop(self, index=-1)}, that removes the
 * element at {@code index} (a negative one counts from the end of the list) and
 * returns it. The element is the one read by {@code self[index]} (see
 * {@link ListModel#read}, that also raises {@code IndexError} if the index
 * might be out of range, and {@code TypeError} if it might not be an
 * {@code int}), and the list is then marked as one whose elements might have
 * been removed (see {@link ListModel#SHRUNK}). The index {@code -1} is in range
 * also when the length of the list is not known, if the list surely has an
 * element: the element is then any of the elements of the list.
 */
public class ListPop extends PyNative {

	protected ListPop(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "pop", params);
	}

	public static ListPop build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListPop(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression list = args[0];
		SymbolicExpression index = missing(args[1]) ? new Constant(Int32Type.INSTANCE, -1, loc) : args[1];
		Shape shape = ListModel.shape(analysis, state, list, st);
		AnalysisState<A> read;
		if (shape.length() == null && shape.prefix() > 0 && shape.elements() != null
				&& isConstant(analysis, state, index, -1))
			// the list surely has an element, but its length is not known
			// (e.g. it has elements at unknown indices): the last element is
			// any of them
			read = ListModel.elements(analysis, state, list, st);
		else
			read = ListModel.read(analysis, state, list, index, st);
		// the list loses an element on the paths where it is read
		return ListModel.shrink(analysis, read, list, st);
	}

	// whether the value is surely the given int
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean isConstant(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			int c)
			throws SemanticException {
		CodeLocation loc = getLocation();
		return hasType(analysis, state, value, INT) == Satisfiability.SATISFIED
				&& analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, value,
						new Constant(Int32Type.INSTANCE, c, loc), ComparisonEq.INSTANCE, loc),
						st) == Satisfiability.SATISFIED;
	}
}
