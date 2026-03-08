package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.*;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.debug.ConstructorResolutionTrace;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class ClassInstantiation extends NaryExpression {

	private final PyClassType classType;

	public ClassInstantiation(
			CFG cfg,
			CodeLocation location,
			PyClassType classType,
			Expression[] params) {
		super(cfg, location, "$ClassInstantiation", classType, params);
		this.classType = classType;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		boolean foundClassRuntimeType = false;
		ExpressionSet returnObj = new ExpressionSet().bottom();
		for (SymbolicExpression identifier : params[0]) {
			Set<Type> runtimeTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, identifier, this);
			for (Type t : runtimeTypes) {
				if (t instanceof PyClassType pct) {
					foundClassRuntimeType = true;
					ResolvedAttributeResolution newResolution = resolveCallableAttribute(interprocedural, state, pct,
							"__new__");
					recordResolution(pct, "__new__", newResolution);
					for (Type _t : newResolution.runtimeTypes()) {
						if (_t instanceof PyFunctionType ft) {
							CodeMember cm = ft.getUnit().getFunction();
							Call c = null;
							if (cm instanceof NativeCFG cfg) {
								c = new NativeCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call",
										List.of(cfg), getSubExpressions());
							} else if (cm instanceof CFG cfg) {
								c = new CFGCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call",
										List.of(cfg), getSubExpressions());
							}
							if (c != null) {
								result = result.lub(c.forwardSemantics(state, interprocedural, expressions));
								ResolvedAttributeResolution initResolution = resolveCallableAttribute(interprocedural,
										result, pct, "__init__");
								CompilationUnit initOwner = initResolution.owner() == null ? pct.getUnit()
										: initResolution.owner();
								returnObj = returnObj.lub(result.getExecutionExpressions());
								PythonScopedAttributeAccessRef unitAttributeAccessRef = new PythonScopedAttributeAccessRef(
										this.getCFG(), getLocation(), initOwner,
										new Global(getLocation(), initOwner, "__init__", false));
								for (SymbolicExpression e : result.getExecutionExpressions()) {
									if (e instanceof Identifier id) {
										Expression[] initExpressions = new Expression[getSubExpressions().length];
										// String syntheticName = id.getName();
										InstrumentedReceiverRef syntheticVariable = new InstrumentedReceiverRef(
												this.getCFG(), getLocation(), false);
										initExpressions[0] = syntheticVariable;
										for (int i = 1; i < getSubExpressions().length; i++)
											initExpressions[i] = getSubExpressions()[i];
										FunctionApply a = new FunctionApply(this.getCFG(), getLocation(),
												unitAttributeAccessRef, initExpressions);
										org.apache.logging.log4j.LogManager.getLogger(ClassInstantiation.class).debug(
												"ClassInstantiation: calling __init__ for {} via FA id={} result.isBottom={}",
												classType, System.identityHashCode(a), result.isBottom());
										result = result.lub(a.forwardSemantics(result, interprocedural, expressions));
									}
								}
							}
						}
					}
					if (newResolution.runtimeTypes().isEmpty())
						result = result.lub(state);
				}
			}
		}

		// If we cannot resolve any runtime class type, keep the incoming state
		// instead of collapsing execution to bottom.
		if (!foundClassRuntimeType)
			return state;
		return result.withExecutionExpressions(returnObj);
	}

	private <A extends AbstractLattice<A>,
			D extends AbstractDomain<A>> ResolvedAttributeResolution resolveCallableAttribute(
					InterproceduralAnalysis<A, D> interprocedural,
					AnalysisState<A> state,
					PyClassType classType,
					String attributeName)
					throws SemanticException {
		LinkedHashSet<String> visited = new LinkedHashSet<>();
		ArrayDeque<CompilationUnit> work = new ArrayDeque<>();
		work.add(classType.getUnit());
		while (!work.isEmpty()) {
			CompilationUnit current = work.removeFirst();
			if (!visited.add(current.getName()))
				continue;
			GlobalVariable variable = new GlobalVariable(Untyped.INSTANCE,
					"$" + current.getName() + "::" + attributeName, getLocation());
			Set<Type> types = interprocedural.getAnalysis().getRuntimeTypesOf(state, variable, this);
			boolean hasCallable = types.stream().anyMatch(PyFunctionType.class::isInstance);
			if (!types.isEmpty() && !types.contains(NoInfoType.INSTANCE) && hasCallable) {
				String mode = visited.size() == 1 ? "direct" : "inherited";
				return new ResolvedAttributeResolution(current, types, String.join(" -> ", visited), mode);
			}

			Collection<CompilationUnit> ancestors = current.getImmediateAncestors();
			if (ancestors.size() > 1)
				return new ResolvedAttributeResolution(null, Set.of(), String.join(" -> ", visited),
						"unsupported-multiple-ancestors");
			// Traverse all ancestors (not just builtins.object) to support
			// multi-level Python inheritance chains such as C2 → C1 → object.
			for (CompilationUnit ancestor : ancestors)
				work.addLast(ancestor);
		}

		return new ResolvedAttributeResolution(null, Set.of(), String.join(" -> ", visited), "unresolved");
	}

	private boolean isObjectUnit(
			CompilationUnit unit) {
		String name = unit.getName();
		return "object".equals(name) || name.endsWith(".object");
	}

	private void recordResolution(
			PyClassType classType,
			String attribute,
			ResolvedAttributeResolution resolution) {
		List<String> ancestors = new ArrayList<>();
		for (CompilationUnit ancestor : classType.getUnit().getImmediateAncestors())
			ancestors.add(ancestor.getName());
		ConstructorResolutionTrace.record(
				getLocation().toString(),
				classType.getUnit().getName(),
				ancestors,
				attribute,
				resolution.lookupPath(),
				resolution.runtimeTypes().toString(),
				resolution.owner() == null ? "<none>" : resolution.owner().getName(),
				resolution.mode());
	}

	private record ResolvedAttributeResolution(
			CompilationUnit owner,
			Set<Type> runtimeTypes,
			String lookupPath,
			String mode) {
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		ClassInstantiation other = (ClassInstantiation) o;
		int cmp = classType.toString().compareTo(other.classType.toString());
		if (cmp != 0)
			return cmp;
		cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}
}