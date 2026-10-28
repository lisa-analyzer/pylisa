package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.CodeMemberType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import it.unive.pylisa.libraries.collections.NamedTuples;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.libraries.sets.SetModel;
import java.util.Collections;
import java.util.Set;

/**
 * Native implementation of Python's {@code type(obj)}: the class of
 * {@code obj}, that is the same value as the name of the class (so that, e.g.,
 * {@code type(x) is int} and {@code type(x) is type(y)} can be decided, see
 * {@code PyBinaryDispatch#identical}). This is the builtin function
 * {@code bool}, {@code int}, {@code float}, {@code str}, {@code bytes},
 * {@code list}, {@code tuple}, {@code set}, {@code dict}, {@code range},
 * {@code zip}, {@code enumerate}, {@code map} or {@code object} for their
 * values (see {@code PyNameRef}), the class {@code NoneType} for {@code None},
 * and the class of an object (see {@code PyTypeLiteral}). The class of any
 * other value is unknown.
 */
public class TypeBuiltin extends PyNative {

	protected TypeBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "type", params);
	}

	public static TypeBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new TypeBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, args[0], this);
		if (types.isEmpty())
			return unknown(analysis, state, Untyped.INSTANCE);
		AnalysisState<A> result = state.bottomExecution();
		for (Type t : types) {
			Constant cls = classOf(t);
			result = result.lub(cls == null ? unknown(analysis, state, Untyped.INSTANCE)
					: compute(analysis, state, cls));
		}
		return result;
	}

	// the class of the values of type t, or null if it is not known
	private Constant classOf(
			Type t) {
		CodeLocation loc = getLocation();
		String builtin = null;
		if (t.isBooleanType())
			builtin = "bool";
		else if (t.isNumericType())
			builtin = t.asNumericType().isIntegral() ? "int" : "float";
		else if (t.isStringType())
			builtin = "str";
		else if (t instanceof PyBytesType)
			builtin = "bytes";
		else if (ListModel.isList(t))
			builtin = "list";
		else if (NamedTuples.isNamedTuple(t))
			// the class created by collections.namedtuple, not known here
			return null;
		else if (Unpacking.isTuple(t))
			builtin = "tuple";
		else if (SetModel.isSet(t))
			builtin = "set";
		else if (DictModel.isDict(t))
			builtin = "dict";
		else if (RangeBuiltin.isRange(t))
			builtin = "range";
		else if (ZipBuiltin.isZip(t))
			builtin = "zip";
		else if (EnumerateBuiltin.isEnumerate(t))
			builtin = "enumerate";
		else if (MapBuiltin.isMap(t))
			builtin = "map";
		else if (ObjectBuiltin.isObject(t))
			builtin = "object";
		if (builtin != null) {
			NativeCFG function = PyNameRef.builtin(getCFG().getProgram(), builtin);
			return new Constant(new CodeMemberType(Collections.singleton(function)), function, loc);
		}
		if (t.isNullType()) {
			// the class of None
			PyClassType none = PyClassType.lookup(LibrarySpecificationProvider.NONE_TYPE);
			return new Constant(new PyTypeTokenType(Collections.singleton(none)), none.getUnit(), loc);
		}

		Type inner = t.isPointerType() ? t.asPointerType().getInnerType() : null;
		if (inner instanceof PyClassType)
			return new Constant(new PyTypeTokenType(Collections.singleton(inner)), ((PyClassType) inner).getUnit(),
					loc);
		return null;
	}
}
