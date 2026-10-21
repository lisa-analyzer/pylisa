package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.PyMro;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.HashSet;
import java.util.Set;

/**
 * The call {@code super().m(args)} inside a method of the class {@code K},
 * whose first parameter is {@code self}: the method {@code m} of the first
 * class after {@code K} in the method resolution order of the class of
 * {@code self} that defines it (see {@link PyMro#definer}), called on
 * {@code self} (that keeps its class). If {@code m} does not accept the
 * arguments, or no class defines it, {@code TypeError} or
 * {@code AttributeError} are raised, if the methods of the classes are all
 * known; the call is unknown otherwise (also for the methods of library
 * classes). {@code object.__init__}, that is
 * called when no class defines {@code __init__}, returns {@code None}.
 */
public class PySuperCall extends UnresolvedCall {

	private final ClassUnit current;

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param current    the class whose method contains the call
	 * @param name       the name of the called method
	 * @param parameters the parameters of the call: {@code self}, and the
	 *                       arguments
	 */
	public PySuperCall(
			CFG cfg,
			CodeLocation location,
			ClassUnit current,
			String name,
			Expression... parameters) {
		super(cfg, location, CallType.INSTANCE, null, name, LeftToRightEvaluation.INSTANCE, parameters);
		this.current = current;
	}

	@Override
	public String toString() {
		return "super()." + super.toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		Set<Type> selfTypes = new HashSet<>();
		for (SymbolicExpression self : params[0])
			selfTypes.addAll(analysis.getRuntimeTypesOf(state, self, this));

		AnalysisState<A> result = state.bottomExecution();
		boolean unknown = selfTypes.isEmpty();
		Set<CompilationUnit> definers = new HashSet<>();
		for (Type t : selfTypes) {
			CompilationUnit unit = t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
					? t.asPointerType().getInnerType().asUnitType().getUnit()
					: null;
			if (unit == null || !unit.isInstanceOf(current)) {
				unknown = true;
				continue;
			}
			CompilationUnit definer = PyMro.definer(unit, getTargetName(), current);
			if (definer != null && !(definer.getLocation() instanceof SourceCodeLocation))
				// a method of a library class, that is not called here
				unknown = true;
			else if (definer != null)
				definers.add(definer);
			else if (!PyBinaryDispatch.fullyKnown(t))
				unknown = true;
			else if (getTargetName().equals("__init__"))
				// object.__init__, that accepts no arguments
				result = result.lub(params.length == 1
						? analysis.smallStepSemantics(state, new PyNoneConstant(loc), this)
						: PyExceptions.raise(analysis, state, getCFG(), loc, this,
								LibrarySpecificationProvider.TYPE_ERROR));
			else
				result = result.lub(PyExceptions.raise(analysis, state, getCFG(), loc, this,
						LibrarySpecificationProvider.ATTRIBUTE_ERROR));
		}

		Set<Type>[] types = parameterTypes(expressions, analysis);
		for (CompilationUnit definer : definers) {
			// the method of the class defining it, called on self
			types[0] = Set.of(new ReferenceType(PyClassType.lookup(definer.getName())));
			Call resolved;
			try {
				resolved = interprocedural.resolve(this, types,
						state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
			} catch (CallResolutionException e) {
				throw new SemanticException("Unable to resolve call " + this, e);
			}
			if (resolved instanceof OpenCall)
				// it does not accept the arguments
				result = result.lub(PyExceptions.raise(analysis, state, getCFG(), loc, this,
						LibrarySpecificationProvider.TYPE_ERROR));
			else {
				result = result.lub(resolved.forwardSemanticsAux(interprocedural, state, params, expressions));
				getMetaVariables().addAll(resolved.getMetaVariables());
			}
		}
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), this));
		return result;
	}
}
