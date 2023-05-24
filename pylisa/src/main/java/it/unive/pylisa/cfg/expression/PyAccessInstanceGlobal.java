package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
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

public class PyAccessInstanceGlobal extends AccessInstanceGlobal {

	public PyAccessInstanceGlobal(CFG cfg, CodeLocation location, Expression receiver, String target) {
		super(cfg, location, receiver, target);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression expr,
					StatementStore<A, H, V, T> expressions) throws SemanticException {
		try {
			PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfref = df.getReference();
			if (expr.getRuntimeTypes(getProgram().getTypes()).stream().anyMatch(t -> (t.equals(dfref)))) {
				String name = getTarget();
				switch (name) {
					case "loc":
					case "iloc":
					case "style":
						// for pandas dataframes we treat some properties as the
						// dataframe itself
						return state.smallStepSemantics(expr, this);
					case "columns":
						// we treat this as a call to keys()
						Keys keys = new Keys(getCFG(), getLocation(), getSubExpression());
						keys.setOriginatingStatement(this);
						return keys.unarySemantics(interprocedural, state, expr, expressions);
				}
			}
		} catch(Exception e) {}
		// FIXME
		AnalysisState<A, H, V, T> sup = super.unarySemantics(interprocedural, state, expr, expressions);
		if (!sup.isBottom())
			return sup;

		Variable var = new Variable(Untyped.INSTANCE, getTarget(), new Annotations(), getLocation());
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, expr, getLocation());
		container.setRuntimeTypes(Untyped.INSTANCE.allInstances(getProgram().getTypes()));
		AccessChild access = new AccessChild(Untyped.INSTANCE, container, var, getLocation());
		return state.smallStepSemantics(access, this);
	}
}
