package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.symbolic.operators.dataframes.Iterate;
import java.util.Set;

public class SequenceGetItem extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceGetItem(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence,
			Expression index) {
		super(cfg, location, constructName, sequence, index);
	}

	public static SequenceGetItem build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceGetItem(cfg, location, "__getitem__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		try {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfref = ((PyClassType) dftype).getReference();
			PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
			Set<Type> rts = state.getState().getRuntimeTypesOf(left, this, state.getState());
			if (rts.stream().anyMatch(dfref::equals)) {
				HeapDereference deref = new HeapDereference(dftype, left, loc);
				UnaryExpression iterate = new UnaryExpression(seriestype, deref, new Iterate(0), loc);
				return state.smallStepSemantics(iterate, st);
			}
		} catch (Exception e) {
			return state.smallStepSemantics(new PushAny(Untyped.INSTANCE, loc), st);
		}
		return state.smallStepSemantics(new PushAny(Untyped.INSTANCE, loc), st);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}
}