package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLine;
import static it.unive.pylisa.PyParsingUtils.getLocation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.lisa.util.frontend.CFGTweaker;
import it.unive.lisa.util.frontend.ControlFlowTracker;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.antlr.PythonParser.Default_assignmentContext;
import it.unive.pylisa.antlr.PythonParser.Function_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.ParamContext;
import it.unive.pylisa.antlr.PythonParser.Param_maybe_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_no_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_with_defaultContext;
import it.unive.pylisa.antlr.PythonParser.ParametersContext;
import it.unive.pylisa.antlr.PythonParser.Star_etcContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.type.PyClassType;

public class PyCodeMemberParser
		extends
		PythonParserBaseVisitor<Object> {

	private static final Logger log = LogManager.getLogger(PyCodeMemberParser.class);

	private final String filePath;

	private final Program program;

	private final Unit unit;

	/**
	 * Builds the parser for a Python CFG.
	 *
	 * @param filePath file path to the file containing the cfg
	 * @param program  the program to which the cfg belongs
	 * @param unit     the unit to which the cfg belongs
	 */
	public PyCodeMemberParser(
			String filePath,
			Program program,
			Unit unit) {
		this.filePath = filePath;
		this.program = program;
		this.unit = unit;
	}

	@Override
	public PyCFG visitFunction_def_raw(
			Function_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic functions are not supported");
		if (ctx.ASYNC() != null)
			log.warn("Async function definitions are not yet supported. The async def at line " + getLine(ctx)
					+ " of file " + filePath + " is unsoundly translated into a def");

		CodeMemberDescriptor descriptor = buildCFGDescriptor(ctx);
		NodeList<CFG, Statement, Edge> list = new NodeList<>(new SequentialEdge());
		Collection<Statement> entrypoints = new HashSet<>();
		// side effects on entrypoints and matrix will affect the cfg
		PyCFG cfg = new PyCFG(descriptor, entrypoints, list);
		fillDefaultsAndHints(ctx, cfg, descriptor);
		ControlFlowTracker control = new ControlFlowTracker();
		PyStatementParser parser = new PyStatementParser(program, filePath, unit, cfg, control);
		ParsedBlock r = parser.visitBlock(ctx.block());
		list.mergeWith(r.getBody());
		entrypoints.add(r.getBegin());
		Function<String, ParsingException> factory = (
				msg) -> new ParsingException(
						"parse error",
						ParsingException.Type.PARSING_ERROR,
						msg,
						getLocation(filePath, ctx));
		CFGTweaker.splitProtectedYields(cfg, factory);
		CFGTweaker.addFinallyEdges(cfg, factory);
		CFGTweaker.addReturns(cfg, factory);
		cfg.simplify();
		return cfg;
	}

	private CodeMemberDescriptor buildCFGDescriptor(
			Function_def_rawContext funcDecl) {
		String funcName = funcDecl.name().getText();

		PyParameter[] cfgArgs = funcDecl.params() != null
				? visitParameters(funcDecl.params().parameters())
				: new PyParameter[0];

		return new CodeMemberDescriptor(
				getLocation(filePath, funcDecl),
				unit,
				unit instanceof ClassUnit ? true : false,
				funcName,
				cfgArgs);
	}

	@Override
	public PyParameter[] visitParameters(
			ParametersContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (ctx.slash_no_default() != null)
			for (Param_no_defaultContext p : ctx.slash_no_default().param_no_default())
				pars.add(buildParameter(p.param(), null, pars.isEmpty()));
		else if (ctx.slash_with_default() != null) {
			for (Param_no_defaultContext p : ctx.slash_with_default().param_no_default())
				pars.add(buildParameter(p.param(), null, pars.isEmpty()));
			for (Param_with_defaultContext p : ctx.slash_with_default().param_with_default())
				pars.add(buildParameter(p.param(), p.default_assignment(), pars.isEmpty()));
		}

		for (Param_no_defaultContext p : ctx.param_no_default())
			pars.add(buildParameter(p.param(), null, pars.isEmpty()));
		for (Param_with_defaultContext p : ctx.param_with_default())
			pars.add(buildParameter(p.param(), p.default_assignment(), pars.isEmpty()));

		if (ctx.star_etc() != null)
			pars.addAll(buildStarEtcParameters(ctx.star_etc()));

		return pars.toArray(PyParameter[]::new);
	}

	private void fillDefaultsAndHints(
			Function_def_rawContext funcDecl,
			PyCFG cfg,
			CodeMemberDescriptor descriptor) {
		if (funcDecl.params() == null)
			return;

		ParametersContext ctx = funcDecl.params().parameters();
		if (ctx.slash_no_default() != null)
			for (Param_no_defaultContext p : ctx.slash_no_default().param_no_default())
				fillDefaultAndHint(p.param(), null, cfg, descriptor);
		else if (ctx.slash_with_default() != null) {
			for (Param_no_defaultContext p : ctx.slash_with_default().param_no_default())
				fillDefaultAndHint(p.param(), null, cfg, descriptor);
			for (Param_with_defaultContext p : ctx.slash_with_default().param_with_default())
				fillDefaultAndHint(p.param(), p.default_assignment(), cfg, descriptor);
		}

		for (Param_no_defaultContext p : ctx.param_no_default())
			fillDefaultAndHint(p.param(), null, cfg, descriptor);
		for (Param_with_defaultContext p : ctx.param_with_default())
			fillDefaultAndHint(p.param(), p.default_assignment(), cfg, descriptor);
	}

	private void fillDefaultAndHint(
			ParamContext param,
			Default_assignmentContext def,
			PyCFG cfg,
			CodeMemberDescriptor descriptor) {
		if (def == null && param.annotation() == null)
			return;

		for (Parameter p : descriptor.getFormals())
			if (p.getName().equals(param.name().getText())) {
				if (param.annotation() != null) {
					// passing null for tracker and control since no new locals
					// and no control flow should be created here
					PyStatementParser parser = new PyStatementParser(program, filePath, unit, cfg, null);
					String typeHint = parser.visitExpression(param.annotation().expression()).toString();
					((PyParameter) p).setTypeHint(typeHint);
				}
				if (def != null) {
					// passing null for tracker and control since no new locals
					// and no control flow should be created here
					PyStatementParser parser = new PyStatementParser(program, filePath, unit, cfg, null);
					Expression defaultValue = parser.visitExpression(def.expression());
					((PyParameter) p).setDefaultValue(defaultValue);
				}
				break;
			}
	}

	private PyParameter buildParameter(
			ParamContext param,
			Default_assignmentContext def,
			boolean first) {
		if (first && unit instanceof ClassUnit)
			// the first parameter of an instance method is 'self': type it
			// with the enclosing class rather than with its annotation
			return new PyParameter(
					getLocation(filePath, param),
					param.name().getText(),
					new ReferenceType(PyClassType.register(unit.getName(), (ClassUnit) unit)));

		return new PyParameter(
				getLocation(filePath, param),
				param.name().getText());
	}

	private List<PyParameter> buildStarEtcParameters(
			Star_etcContext ctx) {
		List<PyParameter> pars = new ArrayList<>();
		if (ctx.param_no_default() != null)
			pars.add(new VarPositionalParameter(getLocation(filePath, ctx.param_no_default()),
					ctx.param_no_default().param().name().getText()));
		else if (ctx.param_no_default_star_annotation() != null)
			pars.add(new VarPositionalParameter(getLocation(filePath, ctx.param_no_default_star_annotation()),
					ctx.param_no_default_star_annotation().param_star_annotation().name().getText()));

		for (Param_maybe_defaultContext p : ctx.param_maybe_default())
			pars.add(new KeywordOnlyParameter(buildParameter(p.param(), p.default_assignment(), false)));

		if (ctx.kwds() != null)
			pars.add(new VarKeywordParameter(getLocation(filePath, ctx.kwds()),
					ctx.kwds().param_no_default().param().name().getText()));

		return pars;
	}
}
