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
import it.unive.lisa.symbolic.value.PushAny;
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

public class AttributeAccess extends UnaryExpression {
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
				resolved = true;
				ResolvedAccess resolvedAccess = resolveModuleMember(interprocedural, state, mt);
				result = result
						.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedAccess.state(),
								resolvedAccess.access(), this));
				continue;
			}
			if (t instanceof PyClassType ct) {
				resolved = true;
				ResolvedAccess resolvedAccess = resolveClassMemberWithMro(interprocedural, state, ct);
				result = result
						.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedAccess.state(),
								resolvedAccess.access(), this));
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
			String owner = null;
			Type runtimeStaticType = expr.getStaticType();
			if (runtimeStaticType instanceof UnknownAttributeType ut)
				owner = ut.getQualifiedName();

			if (owner == null) {
				Type declaredStaticType = getSubExpression().getStaticType();
				if (declaredStaticType instanceof UnknownAttributeType ut)
					owner = ut.getQualifiedName();
			}

			if (owner == null) {
				String receiver = getSubExpression().toString();
				if (receiver.startsWith("$"))
					receiver = receiver.substring(1);
				owner = receiver.replace("::", ".");
			}

			ResolvedAccess resolvedUnknown = resolveUnknownMember(interprocedural, state, owner);
			result = result.lub(interprocedural.getAnalysis().smallStepSemantics(resolvedUnknown.state(),
					resolvedUnknown.access(), this));
		}

		return result;
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

		AnalysisState<A> unknownValue = interprocedural.getAnalysis().smallStepSemantics(updated,
				new PushAny(UnknownAttributeType.lookup(owner + "." + target), getLocation()),
				this);
		AnalysisState<A> assigned = updated.bottom();
		for (SymbolicExpression expression : unknownValue.getExecutionExpressions())
			assigned = assigned.lub(interprocedural.getAnalysis().assign(unknownValue, unknown, expression, this));

		return new ResolvedAccess(assigned, unknown);
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

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ResolvedAccess resolveClassMemberWithMro(
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

		String owner = classType.getUnit().getName();
		String unknownName = UnknownSymbolUtils.unknownAttributeName(owner, target);
		AnalysisState<A> updated = registerUnknownMember(state, owner, "$" + owner + "::" + target);
		GlobalVariable unknown = new GlobalVariable(UnknownAttributeType.lookup(owner + "." + target),
				unknownName,
				getLocation());
		return new ResolvedAccess(updated, unknown);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> registerUnknownMember(
			AnalysisState<A> state,
			String owner,
			String valueName) {
		if (state.getExecutionInfo(ObjectRegister.INFO_KEY) == null)
			return state;
		ObjectRegister register = state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class);
		//System.out.println("WHY YOU DID THIS? YOU ARE STORING THIS $builtins.object.super([__main__.CSup, $self])::__init__ IN THE OBJECT REGISTER. ALSO IN THE HEAP STATE: $builtins.object.super([__main__.CSup, $self])::__init__: [heap[s]:pp@unknown@'py-testcases/classes/classes_super.py':5:16]");
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
