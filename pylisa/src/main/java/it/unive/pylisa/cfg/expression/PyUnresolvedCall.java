package it.unive.pylisa.cfg.expression;

import java.util.Arrays;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.symbols.Aliases;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.QualifierSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.statement.SuperCallNoArgs;
import it.unive.pylisa.cfg.type.PyClassType;

public class PyUnresolvedCall extends UnresolvedCall {

	public PyUnresolvedCall(
			CFG cfg,
			CodeLocation location,
			String targetName,
			Expression... parameters) {
		super(cfg, location, CallType.UNKNOWN, null, targetName, LeftToRightEvaluation.INSTANCE, Untyped.INSTANCE,
				parameters);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CFG cfg = getCFG();
		CodeMemberDescriptor descriptor = cfg.getDescriptor();
		CodeLocation loc = getLocation();
		Program program = descriptor.getUnit().getProgram();
		String fname = getTargetName();
		SymbolAliasing aliasing = state.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);
		Unit cu;

		if (fname.equals("super") && params.length == 0 && descriptor.isInstance()) {
			// super() might be a call to a user-defined function, or to the
			// built-in super() for which we have default parameters and a
			// native implementation
			if (descriptor.getFormals().length == 0)
				throw new SemanticException(
						"An instance CFG should always have the receiver as first parameter, but no parameters were found");

			Expression[] formals = new Expression[2];
			formals[0] = new PyTypeLiteral(cfg, loc, descriptor.getUnit());
			formals[1] = new VariableRef(cfg, loc, descriptor.getFormals()[0].getName());
			return new SuperCallNoArgs(cfg, loc, formals)
					.forwardSemantics(state, interprocedural, expressions);
		}

		for (SymbolicExpression expr : params[0]) {
			Expression[] formals = getParameters();
			ExpressionSet[] actuals = params;

			// case 1: Foo.__init__(...)
			if ((cu = program.getUnit(fname)) == null) {
				// case 2: library.Foo.__init__(...)
				String class_candidate = expr.toString().replace("::", ".");
				if ((cu = program.getUnit(class_candidate + "." + fname)) == null && aliasing != null) {
					Aliases qAlias = aliasing.getState(new QualifierSymbol(class_candidate));
					Aliases qnAlias = aliasing.getState(new QualifiedNameSymbol(class_candidate, fname));

					// we first check the qualified name, then the qualifier
					// individually
					if (!qnAlias.isEmpty())
						for (QualifiedNameSymbol alias : qnAlias.castElements(QualifiedNameSymbol.class))
							if ((cu = program.getUnit(alias.getQualifier() + "." + alias.getName())) != null)
								break;

					if (cu == null && !qAlias.isEmpty())
						for (QualifierSymbol alias : qAlias.castElements(QualifierSymbol.class))
							if ((cu = program.getUnit(alias.getQualifier() + "." + fname)) != null)
								break;
				}

				if (cu != null) {
					// if cu is not null here but it was null before entering
					// this block, we used the first parameter as a
					// qualifier of the call: it should be removed from the
					// parameters list
					formals = Arrays.copyOfRange(formals, 1, formals.length);
					actuals = Arrays.copyOfRange(actuals, 1, actuals.length);
				}
			}

			if (cu != null && cu instanceof ClassUnit)
				return new PyNewObj(cfg, loc, PyClassType.lookup(cu.getName()), formals)
						.forwardSemanticsAux(interprocedural, state, params, expressions);
		}

		// default behavior otherwise
		return super.forwardSemanticsAux(interprocedural, state, params, expressions);
	}

}
