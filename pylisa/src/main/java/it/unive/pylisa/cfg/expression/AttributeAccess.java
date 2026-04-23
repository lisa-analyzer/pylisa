package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.string.StringConstant;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.statement.UnknownSymbolUtils;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.type.UnknownAttributeType;
import java.util.ArrayDeque;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class AttributeAccess extends UnaryExpression {
	private static final Logger LOG = LogManager.getLogger(AttributeAccess.class);
	private static final AtomicInteger BIG_TYPES_COUNT = new AtomicInteger(0);
	String target;

	public String getTarget() {
		return target;
	}

	public AttributeAccess(
			CFG cfg,
			CodeLocation location,
			Expression access,
			String target) {
		super(cfg, location, "$AttributeAccess", access);
		this.target = target;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type> runTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, expr, this);
		if (target.equals("include_router") || target.equals("get") || target.equals("post")) {
			LOG.info("[AA2-TRACK] .{} on {} runTypes.size={} first-class={} PyClassNames={}", target,
					getSubExpression(),
					runTypes.size(),
					runTypes.isEmpty() ? "none" : runTypes.iterator().next().getClass().getSimpleName(),
					runTypes.stream().filter(t -> t instanceof PyClassType)
							.map(t -> ((PyClassType) t).getUnit().getName())
							.limit(5).toList());
		}
		AnalysisState<A> result = state.bottom();
		boolean resolved = false;
		for (Type t : runTypes) {
			if (t instanceof ReferenceType rt) {
				AnalysisState<A> instanceAccess = resolveInstanceMember(interprocedural, state, expr);
				if (!instanceAccess.isBottom() && hasResolvedInstanceType(interprocedural, instanceAccess)) {
					resolved = true;
					result = result.lub(instanceAccess);
					continue;
				}
				// Instance access was bottom or yielded only unresolved types —
				// fall through to class MRO
				t = rt.getInnerType();
			}
			if (t instanceof PyModuleType mt) {
				// Only count as resolved when the module actually has the
				// member; otherwise let the outer !resolved path emit a single
				// generic UAT. Registering a UAT per (module × target) here
				// would inflate state by O(modules × attribute_sites) per pass.
				ResolvedAccess resolvedAccess = tryResolveModuleMember(interprocedural, state, mt);
				if (resolvedAccess != null) {
					resolved = true;
					result = result
							.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedAccess.state(),
									resolvedAccess.access(), this));
				}
				continue;
			}
			if (t instanceof PyClassType ct) {
				// Same rationale as above: only mark resolved if some class in
				// the MRO actually defines the member. Otherwise, skip; the
				// outer fallback emits a single generic UAT.
				ResolvedAccess resolvedAccess = tryResolveClassMemberWithMro(interprocedural, state, ct);
				if (resolvedAccess != null) {
					resolved = true;
					result = result
							.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedAccess.state(),
									resolvedAccess.access(), this));
				}
				continue;
			}
			if (t instanceof UnknownAttributeType ut) {
				resolved = true;
				ResolvedAccess resolvedUnknown = resolveUnknownMember(interprocedural, state, ut.getQualifiedName());
				result = result.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedUnknown.state(),
						resolvedUnknown.access(), this));
			}
		}

		if (!resolved) {
			// Library-method fallback: when the receiver's runtime type set is
			// empty or carries no PyClassType (observed for variables in deep
			// CBA sub-contexts whose PyAssign binding didn't propagate), try
			// to resolve `<target>` as an instance method on known library
			// classes. This lets `api_router.include_router(...)` or
			// `@router.get(...)` dispatch to the pluggable even when
			// `api_router`/`router` itself has no tracked type in this state.
			ResolvedAccess libFallback = tryLibraryMethodFallback(interprocedural, state);
			if (libFallback != null) {
				resolved = true;
				result = result.lub(interprocedural.getAnalysis().smallStepSemantics(libFallback.state(),
						libFallback.access(), this));
			}
		}
		if (!resolved) {
			String owner = deriveUnknownOwner(expr);
			ResolvedAccess resolvedUnknown = resolveUnknownMember(interprocedural, state, owner);
			result = result.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedUnknown.state(),
					resolvedUnknown.access(), this));
		}

		return result;
	}

	private String deriveUnknownOwner(
			SymbolicExpression expr) {
		Type runtimeStaticType = expr.getStaticType();
		if (runtimeStaticType instanceof UnknownAttributeType ut)
			return ut.getQualifiedName();

		Type declaredStaticType = getSubExpression().getStaticType();
		if (declaredStaticType instanceof UnknownAttributeType ut)
			return ut.getQualifiedName();

		String receiver = getSubExpression().toString();
		if (receiver.startsWith("$"))
			receiver = receiver.substring(1);
		return receiver.replace("::", ".");
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> resolveInstanceMember(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression receiver)
			throws SemanticException {
		Variable field = new Variable(Untyped.INSTANCE, target, new Annotations(), getLocation());
		HeapDereference container = new HeapDereference(Untyped.INSTANCE, receiver, getLocation());
		AccessChild access = new AccessChild(Untyped.INSTANCE, container, field, getLocation());
		return interprocedural.getAnalysis().smallStepSemantics(state, access, this);
	}

	/**
	 * Library-method fallback for receivers with no resolved runtime type.
	 * <p>
	 * Walks a whitelist of library classes (fastapi, flask) and checks whether
	 * {@code <libClass>.<target>} is registered as a PyFunctionType. If so,
	 * returns a GlobalVariable whose static type is that PyFunctionType, which
	 * lets the outer FunctionApply dispatch to the library pluggable.
	 * <p>
	 * Heuristic: this trades precision (any method matching the target name on
	 * any whitelisted class will dispatch) for recall (endpoints that would
	 * otherwise be invisible due to lost type info now appear). Whitelisted
	 * classes are the ones whose method vocabulary overlaps with the typical
	 * "router"/"app" idiom.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess tryLibraryMethodFallback(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state)
			throws SemanticException {
		// Only applied for the subset of method names that are commonly used
		// as library instance methods in the network-analysis domain.
		switch (target) {
		case "include_router":
		case "get":
		case "post":
		case "put":
		case "delete":
		case "patch":
		case "head":
		case "options":
		case "trace":
		case "add_middleware":
		case "middleware":
		case "mount":
		case "route":
		case "add_api_route":
		case "add_route":
		case "exception_handler":
			break;
		default:
			return null;
		}
		String[] candidates = { "fastapi.APIRouter", "fastapi.FastAPI", "flask.Flask", "flask.Blueprint" };
		for (String cls : candidates) {
			String qualified = cls + "." + target;
			if (it.unive.pylisa.cfg.type.PyFunctionType.isRegistered(qualified)) {
				it.unive.pylisa.cfg.type.PyFunctionType pft = it.unive.pylisa.cfg.type.PyFunctionType
						.lookup(qualified);
				GlobalVariable access = new GlobalVariable(pft, "$" + qualified, getLocation());
				return new ResolvedAccess(state, access);
			}
		}
		return null;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess resolveUnknownMember(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			String owner)
			throws SemanticException {
		String unknownName = UnknownSymbolUtils.unknownAttributeName(owner, target);
		AnalysisState<A> updated = registerUnknownMember(state, owner, unknownName);
		GlobalVariable unknown = new GlobalVariable(UnknownAttributeType.lookup(owner + "." + target),
				unknownName,
				getLocation());
		return new ResolvedAccess(updated, unknown);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess resolveModuleMember(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			PyModuleType moduleType)
			throws SemanticException {
		StringConstant m = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class)
				.getState(moduleType.getUnit().getName());
		String qualifier = m.isBottom() ? "$" + moduleType.getUnit().getName() : m.value;
		GlobalVariable direct = new GlobalVariable(Untyped.INSTANCE, qualifier + "::" + target, getLocation());
		Set<Type> directTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, direct, this);
		if (!UnknownSymbolUtils.isUnresolvedTypeSet(directTypes))
			return new ResolvedAccess(state, direct);
		String owner = moduleType.getUnit().getName();
		String unknownName = UnknownSymbolUtils.unknownAttributeName(owner, target);
		AnalysisState<A> updated = registerUnknownMember(state, owner, qualifier + "::" + target);
		GlobalVariable unknown = new GlobalVariable(UnknownAttributeType.lookup(owner + "." + target),
				unknownName,
				getLocation());
		return new ResolvedAccess(updated, unknown);
	}

	/**
	 * Like {@link #resolveModuleMember} but returns {@code null} when the
	 * member is not found, instead of registering a UAT. Used when iterating
	 * over a large runtime-type set where registering a UAT per (module ×
	 * target) pair would inflate state across CBA passes.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess tryResolveModuleMember(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			PyModuleType moduleType)
			throws SemanticException {
		StringConstant m = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class)
				.getState(moduleType.getUnit().getName());
		String qualifier = m.isBottom() ? "$" + moduleType.getUnit().getName() : m.value;
		GlobalVariable direct = new GlobalVariable(Untyped.INSTANCE, qualifier + "::" + target, getLocation());
		Set<Type> directTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, direct, this);
		if (!UnknownSymbolUtils.isUnresolvedTypeSet(directTypes))
			return new ResolvedAccess(state, direct);
		return null;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess resolveClassMemberWithMro(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			PyClassType classType)
			throws SemanticException {
		ResolvedAccess found = tryResolveClassMemberWithMro(interprocedural, state, classType);
		if (found != null)
			return found;
		String owner = classType.getUnit().getName();
		String unknownName = UnknownSymbolUtils.unknownAttributeName(owner, target);
		AnalysisState<A> updated = registerUnknownMember(state, owner, "$" + owner + "::" + target);
		GlobalVariable unknown = new GlobalVariable(UnknownAttributeType.lookup(owner + "." + target),
				unknownName,
				getLocation());
		return new ResolvedAccess(updated, unknown);
	}

	/**
	 * Like {@link #resolveClassMemberWithMro} but returns {@code null} when no
	 * class in the MRO defines the member — without registering a UAT. The
	 * UAT-per-(class × target) registration was the cause of a state explosion
	 * (and thus slow CBA convergence) when `getRuntimeTypesOf` returned every
	 * registered class for an untyped receiver. The outer caller can emit a
	 * single generic UAT once, after iterating all runtime types.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess tryResolveClassMemberWithMro(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			PyClassType classType)
			throws SemanticException {
		ArrayDeque<CompilationUnit> queue = new ArrayDeque<>();
		LinkedHashSet<String> seen = new LinkedHashSet<>();
		queue.add(classType.getUnit());
		while (!queue.isEmpty()) {
			CompilationUnit current = queue.removeFirst();
			if (!seen.add(current.getName()))
				continue;

			StringConstant m = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class)
					.getState(current.getName());
			String qualifier = m.isBottom() ? "$" + current.getName() : m.value;
			GlobalVariable candidate = new GlobalVariable(Untyped.INSTANCE, qualifier + "::" + target, getLocation());
			Set<Type> candidateTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, candidate, this);
			if (!UnknownSymbolUtils.isUnresolvedTypeSet(candidateTypes))
				return new ResolvedAccess(state, candidate);

			for (CompilationUnit ancestor : current.getImmediateAncestors())
				queue.addLast(ancestor);
		}
		return null;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> registerUnknownMember(
			AnalysisState<A> state,
			String owner,
			String valueName) {
		if (state.getExecutionInfo(ObjectRegister.INFO_KEY) == null)
			return state;
		ObjectRegister register = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class);
		// System.out.println("WHY YOU DID THIS? YOU ARE STORING THIS
		// $builtins.object.super([__main__.CSup, $self])::__init__ IN THE
		// OBJECT REGISTER. ALSO IN THE HEAP STATE:
		// $builtins.object.super([__main__.CSup, $self])::__init__:
		// [heap[s]:pp@unknown@'py-testcases/classes/classes_super.py':5:16]");
		return state.storeExecutionInfo(ObjectRegister.INFO_KEY,
				register.putModule(owner + "." + target, new StringConstant(valueName)));
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean hasResolvedInstanceType(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> instanceAccess) {
		for (SymbolicExpression se : instanceAccess.getExecutionExpressions()) {
			try {
				Set<Type> types = interprocedural.getAnalysis().getRuntimeTypesOf(instanceAccess, se, this);
				if (!UnknownSymbolUtils.isUnresolvedTypeSet(types))
					return true;
			} catch (SemanticException e) {
				// ignore — treat as unresolved
			}
		}
		return false;
	}

	@Override
	public String toString() {
		return getSubExpression() + "::" + target;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	private static class ResolvedAccess {
		private final AnalysisState<?> state;
		private final GlobalVariable access;

		private ResolvedAccess(
				AnalysisState<?> state,
				GlobalVariable access) {
			this.state = state;
			this.access = access;
		}

		@SuppressWarnings("unchecked")
		private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> state() {
			return (AnalysisState<A>) state;
		}

		private GlobalVariable access() {
			return access;
		}
	}

}
