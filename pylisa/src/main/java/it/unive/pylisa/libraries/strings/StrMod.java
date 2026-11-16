package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import it.unive.pylisa.symbolic.operators.value.StringFormatRaises;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Native implementation of {@code str.__mod__(self, other)} and
 * {@code bytes.__mod__(self, other)}: printf-style formatting of {@code self}
 * with {@code other} ({@code "%s" % x}, {@code b"%d" % 5}). It raises
 * {@code TypeError} or {@code ValueError} when Python does (e.g.
 * {@code "ab" % 7}, {@code "%q" % 7}, or {@code b"%s" % "x"}), as decided by
 * the domains through {@link StringFormatRaises}, or as decided here for a
 * constant format whose conversions surely accept the arguments (e.g.
 * {@code "%d" % n} for an {@code int} {@code n}). There is no {@code __rmod__}:
 * formatting is always driven by the left-hand format.
 */
public class StrMod extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected StrMod(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static StrMod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrMod(cfg, location, "__mod__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();

		// whether the formatting raises is decided by the domains that can
		// evaluate it, while all others answer that it might
		it.unive.lisa.symbolic.value.BinaryExpression typeError = new it.unive.lisa.symbolic.value.BinaryExpression(
				BoolType.INSTANCE, left, right, StringFormatRaises.TYPE_ERROR, loc);
		it.unive.lisa.symbolic.value.BinaryExpression valueError = new it.unive.lisa.symbolic.value.BinaryExpression(
				BoolType.INSTANCE, left, right, StringFormatRaises.VALUE_ERROR, loc);
		Satisfiability raisesTypeError = analysis.satisfies(state, typeError, context, this);
		Satisfiability raisesValueError = analysis.satisfies(state, valueError, context, this);
		if ((raisesTypeError != Satisfiability.NOT_SATISFIED || raisesValueError != Satisfiability.NOT_SATISFIED)
				&& accepted(analysis, context, state, left, right)) {
			raisesTypeError = Satisfiability.NOT_SATISFIED;
			raisesValueError = Satisfiability.NOT_SATISFIED;
		}

		// bytes formats produce bytes
		Type type = analysis.getRuntimeTypesOf(state, left, context, this).stream()
				.anyMatch(t -> t instanceof PyBytesType) ? PyBytesType.INSTANCE : StringType.INSTANCE;
		AnalysisState<A> result = state.bottomExecution();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		// the formatting succeeds where neither exception is raised, and each
		// exception is raised exactly where its condition holds
		if (raisesTypeError != Satisfiability.SATISFIED && raisesValueError != Satisfiability.SATISFIED) {
			AnalysisState<A> formatting = state;
			if (raisesTypeError == Satisfiability.UNKNOWN)
				formatting = ExceptionGuard.unless(analysis, context, formatting, typeError, this);
			if (raisesValueError == Satisfiability.UNKNOWN)
				formatting = ExceptionGuard.unless(analysis, context, formatting, valueError, this);
			if (!formatting.getExecutionState().isBottom())
				result = result.lub(analysis.smallStepSemantics(formatting,
						new it.unive.lisa.symbolic.value.BinaryExpression(type, left, right, StringFormat.INSTANCE,
								loc),
						context, st));
		}
		if (raisesTypeError != Satisfiability.NOT_SATISFIED)
			raising.raise(raisesTypeError == Satisfiability.SATISFIED ? state
					: ExceptionGuard.when(analysis, context, state, typeError, this),
					getCFG(), loc, st, LibrarySpecificationProvider.TYPE_ERROR);
		if (raisesValueError != Satisfiability.NOT_SATISFIED)
			raising.raise(raisesValueError == Satisfiability.SATISFIED ? state
					: ExceptionGuard.when(analysis, context, state, valueError, this),
					getCFG(), loc, st, LibrarySpecificationProvider.VALUE_ERROR);
		return raising.moveFrom(result);
	}

	// whether the format is a str constant whose conversions surely accept
	// the arguments: there are as many arguments as conversions (a tuple of
	// known length, or a single value that is not a tuple), and each
	// argument has a type accepted by its conversion
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean accepted(
			Analysis<A, D> analysis,
			ScopeId<A> context,
			AnalysisState<A> state,
			SymbolicExpression format,
			SymbolicExpression args)
			throws SemanticException {
		if (!(format instanceof Constant) || !(((Constant) format).getValue() instanceof String))
			return false;
		List<Character> conversions = conversions((String) ((Constant) format).getValue());
		if (conversions == null)
			return false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, args, context, this);
		if (types.isEmpty() || types.stream().anyMatch(Type::isUntyped))
			return false;
		List<SymbolicExpression> values;
		if (types.stream().allMatch(Unpacking::isTuple)) {
			values = ListModel.exactElements(analysis, context, state, args, this);
			if (values == null)
				return false;
		} else if (types.stream().noneMatch(t -> Unpacking.isTuple(t) || DictModel.isDict(t)))
			values = List.of(args);
		else
			return false;
		if (values.size() != conversions.size())
			return false;
		for (int i = 0; i < values.size(); i++) {
			Set<Type> argTypes = analysis.getRuntimeTypesOf(state, values.get(i), context, this);
			char conversion = conversions.get(i);
			if (argTypes.isEmpty() || !argTypes.stream().allMatch(t -> accepts(conversion, t)))
				return false;
		}
		return true;
	}

	// whether the conversion surely accepts a value of the given type (the
	// conversions of str and repr only accept builtin values, as those of
	// objects might raise)
	private static boolean accepts(
			char conversion,
			Type t) {
		switch (conversion) {
		case 's':
		case 'r':
		case 'a':
			return PyBinaryDispatch.isBuiltinValueType(t);
		case 'o':
		case 'x':
		case 'X':
			return PyNative.INT.test(t);
		default:
			// d, i, u, e, E, f, F, g, G
			return t.isNumericType() || t.isBooleanType();
		}
	}

	// the conversions of a printf-style format, in order, or null if it has
	// conversions that are not supported here (mapping keys, * widths, %c, or
	// invalid ones)
	private static List<Character> conversions(
			String format) {
		List<Character> result = new ArrayList<>();
		int n = format.length();
		for (int i = 0; i < n; i++) {
			if (format.charAt(i) != '%')
				continue;
			i++;
			if (i >= n || format.charAt(i) == '(')
				return null;
			while (i < n && "#0- +".indexOf(format.charAt(i)) >= 0)
				i++;
			while (i < n && Character.isDigit(format.charAt(i)))
				i++;
			if (i < n && format.charAt(i) == '.') {
				i++;
				while (i < n && Character.isDigit(format.charAt(i)))
					i++;
			}
			while (i < n && "hlL".indexOf(format.charAt(i)) >= 0)
				i++;
			if (i >= n)
				return null;
			char c = format.charAt(i);
			if (c == '%')
				continue;
			if ("diouxXeEfFgGrsa".indexOf(c) < 0)
				return null;
			result.add(c);
		}
		return result;
	}
}
