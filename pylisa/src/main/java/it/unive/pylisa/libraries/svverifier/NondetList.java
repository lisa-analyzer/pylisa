package it.unive.pylisa.libraries.svverifier;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Native implementation of {@code _sv_verifier.nondet_list(nondet_elem)}, that
 * returns a new list with any number of elements, each produced by calling
 * {@code nondet_elem()}. Since the number of elements is unknown, all of them
 * are elements at unknown indices of the list (see
 * {@link ListCreation#UNKNOWN_INDEX}), and the length of the list is unknown.
 * The elements are the results of calling {@code nondet_elem} when it is a
 * function of the program (whose exceptions might be raised), or values of the
 * corresponding type when it is one of the {@code nondet_*} functions of
 * {@code _sv_verifier} (e.g. {@code _sv_verifier.nondet_int}); otherwise, they
 * are unknown. The list might also be empty, when {@code nondet_elem} is never
 * called.
 */
public class NondetList extends PyNative {

	// the types of the values returned by the nondet_* functions
	private static final Map<String, Type> NONDET_TYPES = Map.of(
			"nondet_bool", BoolType.INSTANCE,
			"nondet_int", Int32Type.INSTANCE,
			"nondet_float", Float32Type.INSTANCE,
			"nondet_str", StringType.INSTANCE);

	protected NondetList(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "nondet_list", params);
	}

	public static NondetList build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NondetList(cfg, location, exprs);
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

		// the elements of the list
		AnalysisState<A> elements = state.bottom();
		Set<Identifier> metas = new HashSet<>();
		Type nondet = nondetType(getSubExpressions()[0]);
		if (nondet != null)
			elements = analysis.smallStepSemantics(state, new PushAny(nondet, loc), st);
		else
			for (SymbolicExpression gen : params[0]) {
				Set<CFG> cfgs = new HashSet<>();
				Set<NativeCFG> natives = new HashSet<>();
				boolean unknown = false;
				Set<Type> types = analysis.getRuntimeTypesOf(state, gen, this);
				if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this))
					unknown = true;
				else
					for (Type t : types)
						if (t.isCodeMemberType())
							for (CodeMember member : t.asCodeMemberType().getMembers())
								if (member instanceof CFG)
									cfgs.add((CFG) member);
								else if (member instanceof NativeCFG)
									natives.add((NativeCFG) member);
								else
									unknown = true;
						else
							unknown = true;
				// nondet_elem is called without arguments
				UnresolvedCall call = new UnresolvedCall(getCFG(), loc, CallType.STATIC, null, "nondet_elem",
						LeftToRightEvaluation.INSTANCE);
				if (!cfgs.isEmpty())
					elements = elements.lub(call(new CFGCall(call, cfgs), interprocedural, state, expressions, metas));
				if (!natives.isEmpty())
					elements = elements
							.lub(call(new NativeCall(call, natives), interprocedural, state, expressions, metas));
				if (unknown)
					elements = elements.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), st));
			}
		// the list is either empty (nondet_elem is never called), or its
		// elements are the values computed above, all at unknown indices
		ListCreation creation = new ListCreation(getCFG(), loc);
		AnalysisState<A> result = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		if (!elements.getExecutionState().isBottom()) {
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, elements, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression list : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				for (SymbolicExpression element : elements.getExecutionExpressions())
					result = result.lub(analysis.assign(created, unknown, element, st).withExecutionExpression(list));
			}
		} else
			// the errors raised by nondet_elem
			result = result.lub(elements);
		getMetaVariables().addAll(creation.getMetaVariables());
		return result.forgetIdentifiers(metas, this);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			Call call,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			StatementStore<A> expressions,
			Set<Identifier> metas)
			throws SemanticException {
		AnalysisState<A> result = call.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0], expressions);
		metas.addAll(call.getMetaVariables());
		return result;
	}

	// the type of the values returned by nondet_elem, if it is one of the
	// nondet_* functions of _sv_verifier (e.g. _sv_verifier.nondet_int)
	private static Type nondetType(
			Expression gen) {
		if (!(gen instanceof UnresolvedCall))
			return null;
		UnresolvedCall access = (UnresolvedCall) gen;
		Expression[] params = access.getParameters();
		if (!access.getTargetName().equals("__getattribute__") || params.length != 2
				|| !(params[0] instanceof VariableRef) || !(params[1] instanceof PyStringLiteral)
				|| !((VariableRef) params[0]).getName().equals("_sv_verifier"))
			return null;
		return NONDET_TYPES.get(((PyStringLiteral) params[1]).getValue());
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// never invoked: forwardSemanticsAux computes the result
		throw new SemanticException("nondet_list is evaluated by forwardSemanticsAux");
	}
}
