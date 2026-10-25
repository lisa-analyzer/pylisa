package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code list.pop(self, index=-1)}, that removes the
 * element at {@code index} (a negative one counts from the end of the list) and
 * returns it. The element is the one read by {@code self[index]} (see
 * {@link ListModel#read}, that also raises {@code IndexError} if the index
 * might be out of range, and {@code TypeError} if it might not be an
 * {@code int}), and the list is then marked as one whose elements might have
 * been removed (see {@link ListModel#SHRUNK}).
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
		SymbolicExpression list = args[0];
		SymbolicExpression index = missing(args[1]) ? new Constant(Int32Type.INSTANCE, -1, getLocation()) : args[1];
		AnalysisState<A> read = ListModel.read(analysis, state, list, index, st);
		// the list loses an element on the paths where it is read
		return ListModel.shrink(analysis, read, list, st);
	}
}
