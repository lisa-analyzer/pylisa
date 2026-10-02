package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.string.StringConstant;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.program.type.UnknownAttributeType;

public class UnknownAttributeSymbolRef extends Expression {

	private final String ownerName;
	private final String memberName;

	public UnknownAttributeSymbolRef(
			CFG cfg,
			CodeLocation location,
			String ownerName,
			String memberName) {
		super(cfg, location, UnknownAttributeType.lookup(ownerName + "." + memberName));
		this.ownerName = ownerName;
		this.memberName = memberName;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return UnknownSymbolUtils.unknownAttributeName(ownerName, memberName);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> state,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		String symbolName = UnknownSymbolUtils.unknownAttributeName(ownerName, memberName);
		if (state.getExecutionInfo(ObjectRegister.INFO_KEY) != null) {
			ObjectRegister register = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class);
			state = state.storeExecutionInfo(ObjectRegister.INFO_KEY,
					register.putModule(ownerName + "." + memberName, new StringConstant(symbolName)));
		}
		GlobalVariable symbol = new GlobalVariable(
				UnknownAttributeType.lookup(ownerName + "." + memberName),
				symbolName,
				getLocation());
		AnalysisState<A> unknownValue = analysis.smallStepSemantics(state,
				new PushAny(UnknownAttributeType.lookup(ownerName + "." + memberName), getLocation()),
				this);
		AnalysisState<A> assigned = state.bottom();
		for (it.unive.lisa.symbolic.SymbolicExpression expression : unknownValue.getExecutionExpressions())
			assigned = assigned.lub(analysis.assign(unknownValue, symbol, expression, this));
		// Empty-fold soundness fallback. `unknownValue` IS the sound ⊤ widening
		// produced by the PushAny on line 66. When the PushAny's execution-
		// expression set is empty under the current heap/type configuration —
		// e.g. for Pydantic-Field-registered attributes whose name pylisa's
		// static class model never saw — the fold above leaves `assigned`
		// at the ⊥ seed. Returning smallStepSemantics(⊥, …) below then
		// poisons the entire downstream analysis with ⊥, even though the
		// concrete program clearly reaches this point. Fall back to assigning
		// `symbol` from PushAny directly so that the next read of `symbol`
		// sees ⊤ (sound) instead of ⊥ (unsound). Discovered while triaging
		// IBM/mcp-context-forge: its Settings(BaseSettings) declares 427
		// Pydantic-Field() attributes, main.py reads ~158 of them, and the
		// first miss bottomed the state for every downstream
		// app.include_router(...).
		if (assigned.isBottom())
			assigned = analysis.assign(state, symbol,
					new PushAny(UnknownAttributeType.lookup(ownerName + "." + memberName), getLocation()),
					this);
		return analysis.smallStepSemantics(assigned, symbol, this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		UnknownAttributeSymbolRef other = (UnknownAttributeSymbolRef) o;
		int cmp = ownerName.compareTo(other.ownerName);
		if (cmp != 0)
			return cmp;
		return memberName.compareTo(other.memberName);
	}
}
