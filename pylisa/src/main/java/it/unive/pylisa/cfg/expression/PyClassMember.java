package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.CodeMemberType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.PyClassParser;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.cfg.PyMro;
import it.unive.pylisa.cfg.expression.PyClassAttributes.ClassAttribute;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.HashSet;
import java.util.Set;

/**
 * The attribute {@code C.name} of a class {@code C} of the program, read as a
 * value (and not called, see {@link PyClassFunctionCall}): if the first class
 * in the method resolution order of {@code C} defining {@code name} (see
 * {@link PyMro#definer}) defines it as a method, its value is the function
 * (e.g. {@code f = C.m}, then {@code f(o)}). Otherwise, its value is the one
 * assigned in the body of that class, if it is a known constant (see
 * {@link PyClassAttributes#attribute}), and unknown if it is not, or if that
 * class is a library one. If no class in the order defines {@code name}, its
 * value is unknown: for {@code mro} and the names of special attributes (e.g.
 * {@code __name__}, {@code __mro__}), that all classes have, and otherwise also
 * {@code AttributeError} might be raised (the attribute might be assigned
 * through the class, as in {@code C.name = v}, or defined by its metaclass). As
 * the target of an assignment ({@code C.name = v}), it is an attribute of a
 * namespace, whose assignment is not tracked (see {@link PyClassAttributes}).
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
		ScopeId<A> context = interprocedural.getCurrentContext();
		if (isWriteTarget())
			// C.name = v: the attributes assigned through classes are not
			// tracked (see PyClassAttributes)
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
		Set<CodeMember> functions = functions();
		if (functions.isEmpty())
			return attribute(interprocedural, state);
		CodeMember function = functions.iterator().next();
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new Constant(new CodeMemberType(functions), function, getLocation()), context, this);
	}

	// the value of the attribute, when it is not a method (see the
	// documentation of this class)
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> attribute(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		AnalysisState<
				A> unknown = analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), context, this);
		Unit unit = getProgram().getUnit(cls);
		if (!ClassBases.isProgramClass(unit))
			return unknown;
		CompilationUnit definer = PyMro.definer((ClassUnit) unit, name, null);
		if (definer == null) {
			if (name.equals("mro") || (name.startsWith("__") && name.endsWith("__")))
				// an attribute that all classes have (e.g. __name__, __mro__),
				// or that object defines (e.g. __init__)
				return unknown;
			// the attribute might be assigned through the class (C.name = v),
			// or defined by its metaclass
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
			raising.raise(state, getCFG(), loc, this, LibrarySpecificationProvider.ATTRIBUTE_ERROR);
			return raising.moveFrom(unknown);
		}
		ClassAttribute attribute = ClassBases.isProgramClass(definer)
				? PyClassAttributes.attribute((ClassUnit) unit, name)
				: null;
		if (attribute == null || attribute.value() == null)
			// defined by a library class, or whose value is not known
			return unknown;
		// the body of the class is not executed: its constant value is
		// computed here
		Expression value = attribute.value();
		AnalysisState<A> result = value.forwardSemantics(state, interprocedural, new StatementStore<>(state));
		// e.g. the return value of the call computing -1: they are forgotten
		// by the statement using this attribute
		getMetaVariables().addAll(value.getMetaVariables());
		return result;
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
