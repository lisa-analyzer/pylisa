package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.pylisa.cfg.PyMro;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The call {@code C.m(args)} of the function {@code m} of a class {@code C} of
 * the program. Through the class, {@code m} is a plain function: it is called
 * with the arguments as they are, with no implicit receiver (e.g.
 * {@code C.m(obj)} passes {@code obj} as {@code self}, and {@code C.m()} raises
 * {@code TypeError} if {@code m} expects {@code self}). If the class does not
 * define {@code m}, the call is resolved as any other call.
 */
public class PyClassFunctionCall extends UnresolvedCall {

	// the names the class might be registered with (e.g. m.C for a class
	// imported with from m import C): imported modules might be parsed after
	// the call, so the class is looked up during the analysis
	private final List<String> classNames;

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param classNames the names the class might be registered with, tried in
	 *                       order
	 * @param name       the name of the function
	 * @param order      the evaluation order of the parameters
	 * @param parameters the parameters of the call
	 */
	public PyClassFunctionCall(
			CFG cfg,
			CodeLocation location,
			List<String> classNames,
			String name,
			EvaluationOrder order,
			Expression... parameters) {
		super(cfg, location, CallType.STATIC, null, name, order, parameters);
		this.classNames = classNames;
	}

	// the class, if it is a class of the program
	private ClassUnit cls() {
		for (String name : classNames) {
			Unit unit = getProgram().getUnit(name);
			if (unit instanceof ClassUnit)
				return (ClassUnit) unit;
		}
		return null;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		ClassUnit cls = cls();
		if (cls == null)
			// e.g. a function of a module
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
		// the function of the first class in the method resolution order
		// defining it
		CompilationUnit definer = PyMro.definer(cls, getTargetName(), null);
		if (definer == null)
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
		Set<CFG> cfgs = new HashSet<>();
		boolean others = add(definer.getInstanceCodeMembersByName(getTargetName(), false), cfgs)
				| add(definer.getCodeMembersByName(getTargetName()), cfgs);
		if (cfgs.isEmpty() || others)
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);

		CFGCall call = CallSites.within(new CFGCall(this, cfgs), this);
		AnalysisState<A> result = call.forwardSemanticsAux(interprocedural, state, params, expressions);
		getMetaVariables().addAll(call.getMetaVariables());
		return result;
	}

	// adds the cfgs among the members, returning whether there are others
	private static boolean add(
			Collection<? extends CodeMember> members,
			Set<CFG> cfgs) {
		boolean others = false;
		for (CodeMember member : members)
			if (member instanceof CFG)
				cfgs.add((CFG) member);
			else
				others = true;
		return others;
	}
}
