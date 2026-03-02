package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.Keys;
import java.util.Set;

public class PyAccessInstanceGlobal extends AccessInstanceGlobal {

	public PyAccessInstanceGlobal(
			CFG cfg,
			CodeLocation location,
			Expression receiver,
			String target) {
		super(cfg, location, receiver, target);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfreftype = dftype.getReference();
			Set<Type> rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, expr, this);
			if (rts.stream().anyMatch(t -> (t.equals(dfreftype))))
				switch (getTarget()) {
				case "loc":
				case "iloc":
				case "style":
					// for pandas dataframes we treat some properties as the
					// dataframe itself
					return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
				case "columns":
					// we treat this as a call to keys()
					Keys keys = new Keys(getCFG(), getLocation(), getSubExpression());
					keys.setOriginatingStatement(this);
					return keys.fwdUnarySemantics(interprocedural, state, expr, expressions);
				}
		}
		// FIXME
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		AnalysisState<A> sup = super.fwdUnarySemantics(interprocedural, state, expr, expressions);
		if (!sup.isBottom())
			return sup;

		Variable var = new Variable(Untyped.INSTANCE, getTarget(), new Annotations(), getLocation());
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, expr, getLocation());
		AccessChild access = new AccessChild(Untyped.INSTANCE, container, var, getLocation());
		return analysis.smallStepSemantics(state, access, this);
	}
}