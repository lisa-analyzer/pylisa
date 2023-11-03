package it.unive.pylisa.libraries.stdlib.object;

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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ObjectGetAttribute extends BinaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectGetAttribute(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg,
				location,
				"__getattribute__",
				PyClassType.lookup(LibrarySpecificationProvider.OBJECT),
				left,
				right);
	}

	public static ObjectGetAttribute build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectGetAttribute(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		String target = getRight().toString();

		// TODO this semantics should actually exploit the current namespace of
		// the receiver: if the property does not exist, we should invoke
		// __getattr__ instead of performing the access

		AccessInstanceGlobal access = new AccessInstanceGlobal(getCFG(), getLocation(), getLeft(), target);
		AnalysisState<A, H, V, T> sup = access.unarySemantics(interprocedural, state, left, expressions);
		if (!sup.isBottom())
			return sup;

		// FIXME fallback for not going to bottom
		Variable var = new Variable(Untyped.INSTANCE, target, new Annotations(), getLocation());
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, left, getLocation());
		container.setRuntimeTypes(Untyped.INSTANCE.allInstances(getProgram().getTypes()));
		AccessChild ac = new AccessChild(Untyped.INSTANCE, container, var, getLocation());
		return state.smallStepSemantics(ac, this);
	}
}
