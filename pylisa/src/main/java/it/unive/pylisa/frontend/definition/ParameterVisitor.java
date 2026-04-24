package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.StarargsContext;
import it.unive.pylisa.antlr.Python3Parser.TfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarkwContext;
import it.unive.pylisa.antlr.Python3Parser.VarpositionalContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

/**
 * Parses Python function-parameter productions ({@code parameters},
 * {@code typedargslist}, {@code starargs}, {@code varkw}, {@code typedarg},
 * {@code tfpdef}, {@code vfpdef}, {@code varargslist}). Extracted from
 * {@link DefinitionVisitor} in Chunk 5.
 */
public final class ParameterVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public ParameterVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public PyParameter[] visitParameters(
			ParametersContext pctx) {
		if (pctx.typedargslist() == null)
			return new PyParameter[0];
		return visitTypedargslist(pctx.typedargslist());
	}

	public PyParameter[] visitTypedargslist(
			TypedargslistContext pctx) {
		List<PyParameter> pars = new LinkedList<>();
		for (TypedargContext typedArg : pctx.typedarg())
			if (pars.isEmpty())
				if (ctx.currentUnit() instanceof ClassUnit) {
					pars.add(new PyParameter(support.getLocation(typedArg), typedArg.tfpdef().NAME().getText(),
							new ReferenceType(PyClassType.register(ctx.currentUnit().getName(),
									(ClassUnit) ctx.currentUnit()))));
				} else
					pars.add(visitTypedarg(typedArg));
			else
				pars.add(visitTypedarg(typedArg));

		if (pctx.starargs() != null)
			pars.addAll(Arrays.asList(visitStarargs(pctx.starargs())));

		if (pctx.varkw() != null)
			pars.add(visitVarkw(pctx.varkw()));

		return pars.toArray(PyParameter[]::new);
	}

	public PyParameter[] visitStarargs(
			StarargsContext pctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (pctx.varpositional() != null) {
			VarpositionalContext def = pctx.varpositional();
			pars.add(new VarPositionalParameter(support.getLocation(def), def.tfpdef().NAME().getText()));
		}

		if (pctx.typedarg() != null) {
			List<TypedargContext> def = pctx.typedarg();
			for (TypedargContext typedArg : def)
				pars.add(new KeywordOnlyParameter(visitTypedarg(typedArg)));
		}
		return pars.toArray(PyParameter[]::new);
	}

	public PyParameter visitVarkw(
			VarkwContext pctx) {
		return new VarKeywordParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText());
	}

	public PyParameter visitTypedarg(
			TypedargContext pctx) {
		String typeHint = null;
		if (pctx.tfpdef().test() != null) {
			typeHint = ctx.expr().visitTest(pctx.tfpdef().test()).toString();
		}
		if (pctx.test() == null)
			return new PyParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText(), Untyped.INSTANCE, null,
					null, typeHint);
		else
			return new PyParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText(), Untyped.INSTANCE,
					ctx.expr().visitTest(pctx.test()), null, typeHint);
	}

	public PyParameter visitTfpdef(
			TfpdefContext pctx) {
		return new PyParameter(support.getLocation(pctx), pctx.NAME().getText(), Untyped.INSTANCE);
	}

	public Object visitVarargslist(
			VarargslistContext pctx) {
		throw new UnsupportedStatementException();
	}

	public String visitVfpdef(
			VfpdefContext pctx) {
		return pctx.NAME().getText();
	}
}
