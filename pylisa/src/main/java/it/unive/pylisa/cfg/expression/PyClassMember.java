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
import it.unive.pylisa.cfg.ClassBases;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.CodeMemberType;
import it.unive.pylisa.PyClassParser;
import it.unive.pylisa.cfg.PyMro;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import java.util.HashSet;
import java.util.Set;

/**
 * The attribute {@code C.name} of a class {@code C} of the program, read as a
 * value (and not called, see {@link PyClassFunctionCall}): if the first class
 * in the method resolution order of {@code C} defining {@code name} (see
 * {@link PyMro#definer}) defines it as a method, its value is the function
 * (e.g. {@code f = C.m}, then {@code f(o)}). Otherwise, it is the read of an
 * attribute of a namespace, as for the other attributes of classes (see
 * {@link PyClassAttributes}).
 */
public class PyClassMember extends UnresolvedCall {

	private final String cls;

	private final String name;

	/**
	 * Builds the read.
	 *
	 * @param cfg      the cfg where the read happens
	 * @param location the location of the read
	 * @param receiver the expression denoting the class
	 * @param cls      the name of the class
	 * @param name     the name of the attribute
	 */
	public PyClassMember(
			CFG cfg,
			CodeLocation location,
			Expression receiver,
			String cls,
			String name) {
		super(cfg, location, CallType.INSTANCE, null, "__getattribute__", receiver,
				new PyStringLiteral(cfg, location, name, "'"));
		this.cls = cls;
		this.name = name;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<CodeMember> functions = isWriteTarget() ? Set.of() : functions();
		if (functions.isEmpty())
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
		CodeMember function = functions.iterator().next();
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new Constant(new CodeMemberType(functions), function, getLocation()), this);
	}

	// the functions of the method name of the class, if it is one
	private Set<CodeMember> functions() {
		Set<CodeMember> functions = new HashSet<>();
		Unit unit = getProgram().getUnit(cls);
		if (!ClassBases.isProgramClass(unit))
			return functions;
		CompilationUnit definer = PyMro.definer((ClassUnit) unit, name, null);
		if (definer == null || !ClassBases.isProgramClass(definer))
			return functions;
		for (CodeMember member : definer.getInstanceCodeMembersByName(name, false))
			if (member instanceof CFG)
				functions.add(member);
		for (CodeMember member : definer.getCodeMembersByName(name))
			if (member instanceof CFG
					&& !member.getDescriptor().getName().equals(PyClassParser.INSTRUMENTED_CLASS_INIT_NAME))
				functions.add(member);
		return functions;
	}

	// whether this read is the target of an assignment (C.name = ...)
	private boolean isWriteTarget() {
		return getParentStatement() instanceof Assignment && ((Assignment) getParentStatement()).getLeft() == this;
	}
}
