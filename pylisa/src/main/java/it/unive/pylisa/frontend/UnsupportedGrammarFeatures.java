package it.unive.pylisa.frontend;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.pylisa.UnsupportedStatementException;
import java.util.Map;
import java.util.Objects;
import org.antlr.v4.runtime.ParserRuleContext;

/**
 * Central registry of Python grammar constructs the PyLiSA front-end does not
 * yet translate. Each entry maps an ANTLR rule-context simple class name
 * ({@code ctx.getClass().getSimpleName()}) to a short, user-facing feature
 * label used in diagnostics.
 * <p>
 * Call-sites must go through
 * {@link #reject(ParserContext, ParserRuleContext, ParserSupport)} (or the
 * {@link ParserSupport#rejectUnsupported} shim) so every rejection emits an
 * {@code UNSUPPORTED} {@link DiagnosticReporter} event <em>and</em> unwinds the
 * visit with {@link UnsupportedStatementException}. Adding support for a listed
 * feature is a three-step change: remove the entry, implement the real visit
 * method, add a unit test.
 */
public final class UnsupportedGrammarFeatures {

	/** ANTLR rule-context simple-class name → human feature label. */
	public static final Map<String, String> FEATURES = Map.ofEntries(
			Map.entry("Yield_exprContext", "yield expression"),
			Map.entry("Yield_argContext", "yield argument"),
			Map.entry("Yield_stmtContext", "yield statement"),
			Map.entry("Star_exprContext", "star expression"),
			Map.entry("Encoding_declContext", "encoding declaration"),
			Map.entry("TrailerContext", "trailer (in unsupported position)"),
			Map.entry("SubscriptlistContext", "subscript list"),
			Map.entry("SliceopContext", "slice step"),
			Map.entry("Comp_forContext", "for-clause of comprehension"),
			Map.entry("Comp_ifContext", "if-clause of comprehension"),
			Map.entry("Comp_iterContext", "comprehension iterator"),
			Map.entry("AnnassignContext", "annotated assignment (standalone)"),
			Map.entry("AugassignContext", "augmented assignment"),
			Map.entry("Global_stmtContext", "global declaration"),
			Map.entry("Nonlocal_stmtContext", "nonlocal declaration"),
			Map.entry("Flow_stmtContext", "flow statement"),
			Map.entry("Raise_stmtContext", "raise statement"),
			Map.entry("Async_stmtContext", "async statement"),
			Map.entry("Except_clauseContext", "typed except clause"),
			Map.entry("Import_as_nameContext", "import-as name"),
			Map.entry("Dotted_as_nameContext", "dotted import-as name"),
			Map.entry("Import_as_namesContext", "import-as list"),
			Map.entry("Dotted_as_namesContext", "dotted import-as list"),
			Map.entry("VarargslistContext", "untyped parameter list"),
			Map.entry("Namedexpr_testContext", ":= walrus in unsupported position"));

	public static String labelFor(
			ParserRuleContext ctx) {
		return FEATURES.get(ctx.getClass().getSimpleName());
	}

	/**
	 * Central rejection path for an unsupported Python construct. Reports an
	 * {@code UNSUPPORTED} event via {@link DiagnosticReporter} (so strict mode
	 * promotes it and the events log captures it), then throws
	 * {@link UnsupportedStatementException} to unwind the visit. The generic
	 * return type lets callers write {@code return reject(...)} from any visit
	 * method irrespective of its declared return type; control never actually
	 * reaches the return since {@code reject} always throws.
	 */
	public static <T> T reject(
			ParserContext pctx,
			ParserRuleContext ctx,
			ParserSupport support) {
		return reject(pctx, ctx, support, Objects.requireNonNullElse(labelFor(ctx), ctx.getClass().getSimpleName()));
	}

	/**
	 * Overload that accepts an explicit feature label, used for feature gaps
	 * detected inside larger visit methods where the triggering rule-context is
	 * too generic to identify via the {@link #FEATURES} map (e.g. the
	 * complex-literal branch inside {@code visitAtom}).
	 */
	public static <T> T reject(
			ParserContext pctx,
			ParserRuleContext ctx,
			ParserSupport support,
			String label) {
		SourceCodeLocation loc = support.getLocation(ctx);
		pctx.reporter().report(
				DiagnosticReporter.Severity.UNSUPPORTED,
				loc,
				label,
				"grammar feature not yet supported by PyLiSA front-end");
		throw new UnsupportedStatementException(label, loc);
	}

	private UnsupportedGrammarFeatures() {
	}
}
