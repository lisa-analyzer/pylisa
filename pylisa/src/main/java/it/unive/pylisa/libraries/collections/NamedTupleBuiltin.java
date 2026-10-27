package it.unive.pylisa.libraries.collections;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.NamedTupleClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

/**
 * Native implementation of
 * {@code collections.namedtuple(typename, field_names, *, rename=False, defaults=None, module=None)}:
 * a new class, whose objects are tuples whose elements are also their
 * attributes, named by the fields (see {@link NamedTupleClassType}). The class
 * is known if {@code typename} is a string literal, and {@code field_names} is
 * a string literal (of names separated by commas or whitespaces) or a list or
 * tuple of string literals, whose names are valid (identifiers, not starting
 * with an underscore, and distinct), with no defaults and no renaming;
 * otherwise, the class is not known, and {@code ValueError} might be raised (as
 * for invalid names).
 */
public class NamedTupleBuiltin extends PyNative {

	// the names that can be fields
	private static final Pattern NAME = Pattern.compile("[A-Za-z][A-Za-z0-9_]*");

	// the keywords of python, that cannot be fields
	private static final Set<String> KEYWORDS = Set.of("False", "None", "True", "and", "as", "assert", "async",
			"await", "break", "class", "continue", "def", "del", "elif", "else", "except", "finally", "for", "from",
			"global", "if", "import", "in", "is", "lambda", "nonlocal", "not", "or", "pass", "raise", "return", "try",
			"while", "with", "yield");

	protected NamedTupleBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "namedtuple", params);
	}

	public static NamedTupleBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NamedTupleBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression rename = args[2], defaults = args[3];
		Expression[] subs = getSubExpressions();
		String name = subs.length > 0 && subs[0] instanceof StringLiteral ? ((StringLiteral) subs[0]).getValue()
				: null;
		List<String> fields = subs.length > 1 ? fields(subs[1]) : null;
		boolean plain = (missing(rename) || analysis.satisfies(state, rename, st) == Satisfiability.NOT_SATISFIED)
				&& (missing(defaults) || hasType(analysis, state, defaults, NONE) == Satisfiability.SATISFIED);
		if (name == null || !valid(name) || fields == null || !plain)
			// the class is not known
			return compute(analysis, state, new PushAny(Untyped.INSTANCE, loc))
					.lub(raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
		NamedTupleClassType type = new NamedTupleClassType(name, fields, loc);
		return compute(analysis, state, new Constant(type, type, loc));
	}

	// the names of the fields, if they are literals and valid, or null
	private static List<String> fields(
			Expression expr) {
		List<String> names = new ArrayList<>();
		if (expr instanceof StringLiteral) {
			for (String part : ((StringLiteral) expr).getValue().replace(',', ' ').trim().split("\\s+"))
				if (!part.isEmpty())
					names.add(part);
		} else if (expr instanceof ListCreation || expr instanceof TupleCreation) {
			for (Expression element : ((NaryExpression) expr).getSubExpressions())
				if (element instanceof StringLiteral)
					names.add(((StringLiteral) element).getValue());
				else
					return null;
		} else
			return null;
		for (String n : names)
			if (!valid(n))
				return null;
		return new HashSet<>(names).size() == names.size() ? names : null;
	}

	private static boolean valid(
			String name) {
		return NAME.matcher(name).matches() && !KEYWORDS.contains(name);
	}
}
