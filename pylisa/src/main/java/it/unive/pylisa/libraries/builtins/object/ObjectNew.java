package it.unive.pylisa.libraries.builtins.object;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.InstrumentedReceiverRef;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Set;

public class ObjectNew extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	private Statement st;

	public ObjectNew(
			CFG cfg,
			CodeLocation location,
			Expression cls) {
		super(cfg, location, "__new__", Untyped.INSTANCE,
				cls);
	}

	public static ObjectNew build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectNew(cfg, location, exprs[0]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		AnalysisState<A> result = state.bottomExecution();
		Set<Type> runtimeTypes = analysis.getRuntimeTypesOf(state, expr, this);
		for (Type runtimeType : runtimeTypes) {
			if (runtimeType instanceof PyClassType classType) {
				MemoryAllocation created = new MemoryAllocation(classType, getLocation(), false);
				HeapReference ref = new HeapReference(new ReferenceType(classType), created, getLocation());
				// allocate the memory region
				AnalysisState<A> allocated = analysis.smallStepSemantics(state, created, this);
				// create the synthetic variable
				InstrumentedReceiverRef newObj = new InstrumentedReceiverRef(getCFG(), getLocation(), false,
						new ReferenceType(classType));
				AnalysisState<A> callState = newObj.forwardSemantics(allocated, interprocedural, expressions);
				AnalysisState<A> tmp = state.bottomExecution();
				for (SymbolicExpression rec : callState.getExecutionExpressions()) {
					tmp = tmp.lub(analysis.assign(callState, rec, ref, newObj));
					if (rec instanceof Identifier i)
						getMetaVariables().add(i);
				}
				result = result.lub(tmp.withExecutionExpressions(callState.getExecutionExpressions()));
			}
		}

		if (!result.isBottom())
			return result;

		// unsound
		return result;
	}
}
