package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLine;
import static it.unive.pylisa.PyParsingUtils.getLocation;

import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.logging.TimerLogger;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.antlr.PythonLexer;
import it.unive.pylisa.antlr.PythonParser;
import it.unive.pylisa.antlr.PythonParser.BlockContext;
import it.unive.pylisa.antlr.PythonParser.Class_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.Default_assignmentContext;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParser.Function_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.ParamContext;
import it.unive.pylisa.antlr.PythonParser.Param_maybe_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_no_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_with_defaultContext;
import it.unive.pylisa.antlr.PythonParser.ParametersContext;
import it.unive.pylisa.antlr.PythonParser.Simple_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Star_etcContext;
import it.unive.pylisa.antlr.PythonParser.StatementContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PyFileParser
		extends
		PythonParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	private static final Logger log = LogManager.getLogger(PyFileParser.class);

	private Map<String, String> imports = new HashMap<>();
	/**
	 * Python program file path.
	 */
	private final String filePath;

	/**
	 * The LiSA program obtained from the Python program at filePath.
	 */
	private final Program program;

	/**
	 * The unit currently under parsing
	 */
	private Unit currentUnit;

	/**
	 * The unit currently under parsing
	 */
	private final CodeUnit rootUnit;

	/**
	 * Current CFG to parse
	 */
	private PyCFG currentCFG;

	private Collection<ControlFlowStructure> cfs;

	/**
	 * Whether or not {@link #filePath} points to a Jupyter notebook file
	 */
	private final boolean notebook;

	/**
	 * List of the indexes of cells of a Jupyter notebook in the order they are
	 * to be executed. Only valid if {@link #notebook} is {@code true}.
	 */
	private final List<Integer> cellOrder;

	/**
	 * Builds the parser for a Python program at {@code filePath}.
	 *
	 * @param program   the LiSA program to which the parsed CFGs will be added
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder list of the indexes of cells of a Jupyter notebook in
	 *                      the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFileParser(
			Program program,
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this.program = program;
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
		this.rootUnit = new CodeUnit(new SourceCodeLocation(filePath, 0, 0),
				program, FilenameUtils.removeExtension(filePath));
		program.addUnit(rootUnit);
		this.currentUnit = rootUnit;
	}

	/**
	 * Returns the parsed unit. Note that the unit will be empty unless
	 * {@link #parse()} is called first.
	 *
	 * @return the parsed unit
	 */
	public CodeUnit getParsedUnit() {
		return rootUnit;
	}

	public CodeUnit parse()
			throws IOException,
			AnalysisSetupException {
		log.info("Reading file... " + filePath);

		PythonLexer lexer = null;
		try (InputStream stream = mkStream();) {
			lexer = new PythonLexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}

		PythonParser parser = new PythonParser(new CommonTokenStream(lexer));
		ParseTree tree = parser.file_input();

		visit(tree);

		return rootUnit;
	}

	private static String transformToCode(
			List<String> code_list) {
		StringBuilder result = new StringBuilder();
		for (String s : code_list)
			result.append(s).append("\n");
		return result.toString();
	}

	private InputStream mkStream() throws FileNotFoundException {
		if (!this.notebook)
			return new FileInputStream(filePath);

		Gson gson = new Gson();
		JsonReader reader = gson.newJsonReader(new FileReader(filePath));
		Map<?, ?> map = gson.fromJson(reader, Map.class);
		List<Map<?, ?>> cells = (ArrayList<Map<?, ?>>) map.get("cells");
		SortedMap<Integer, String> codeBlocks = new TreeMap<>();
		for (int i = 0; i < cells.size(); i++) {
			Map<?, ?> cell = cells.get(i);
			String ctype = (String) cell.get("cell_type");
			if (ctype.equals("code")) {
				List<String> code_list = (List<String>) cell.get("source");
				codeBlocks.put(i, transformToCode(code_list));
			}
		}

		StringBuilder code = new StringBuilder();

		if (cellOrder.isEmpty())
			for (Entry<Integer, String> c : codeBlocks.entrySet())
				code.append(c.getValue()).append("\n");
		else {
			log.warn("The following cells contain code and can be analyzed: " + codeBlocks.keySet());
			for (int idx : cellOrder) {
				String str = codeBlocks.get(idx);
				if (str == null)
					log.warn("Cell " + idx + " does not contain code and will be skipped");
				else
					code.append(str).append("\n");
			}
		}

		return new ByteArrayInputStream(code.toString().getBytes());
	}

	@Override
	public Object visit(
			ParseTree tree) {
		if (tree instanceof File_inputContext)
			return visitFile_input((File_inputContext) tree);

		throw new UnsupportedOperationException("Unsupported parse tree: " + tree.getClass().getSimpleName());
	}

	@Override
	public PyCFG visitFile_input(
			File_inputContext ctx) {
		currentCFG = new PyCFG(buildMainCFGDescriptor(getLocation(filePath, ctx)));
		cfs = new HashSet<>();
		currentUnit.addCodeMember(currentCFG);
		PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, currentCFG);
		ParsedBlock parsed = TimerLogger.execSupplier(log, "Parsing statement list of " + filePath,
				() -> parser.visitStatements(ctx.statements()));
		currentCFG.getNodeList().mergeWith(parsed.getBody());
		currentCFG.getEntrypoints().add(parsed.getBegin());
		addRetNodes(currentCFG);
		cfs.forEach(currentCFG.getDescriptor()::addControlFlowStructure);
		currentCFG.simplify();
		return currentCFG;
	}

	private void addRetNodes(
			PyCFG cfg) {
		Ret ret = new Ret(cfg, cfg.getDescriptor().getLocation());
		if (cfg.getNodesCount() == 0) {
			// empty method, so the ret is also the entrypoint
			cfg.addNode(ret, true);
		} else {
			// every non-throwing instruction that does not have a follower
			// is ending the method
			Collection<Statement> preExits = new LinkedList<>();
			for (Statement st : cfg.getNodes())
				if (!st.stopsExecution() && cfg.followersOf(st).isEmpty())
					preExits.add(st);
			if (!preExits.isEmpty()) {
				cfg.addNode(ret);
				for (Statement st : preExits)
					cfg.addEdge(new SequentialEdge(st, ret));
				for (VariableTableEntry entry : cfg.getDescriptor().getVariables())
					if (preExits.contains(entry.getScopeEnd()))
						entry.setScopeEnd(ret);
			}
		}
	}

	private CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};

		return new CodeMemberDescriptor(loc, currentUnit, false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

	private CodeMemberDescriptor buildCFGDescriptor(
			Function_def_rawContext funcDecl) {
		String funcName = funcDecl.name().getText();

		PyParameter[] cfgArgs = funcDecl.params() != null
				? visitParameters(funcDecl.params().parameters())
				: new PyParameter[0];

		return new CodeMemberDescriptor(getLocation(filePath, funcDecl), currentUnit,
				currentUnit instanceof ClassUnit ? true : false,
				funcName, cfgArgs);
	}

	@Override
	public PyCFG visitFunction_def_raw(
			Function_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic functions are not supported");
		if (ctx.ASYNC() != null)
			log.warn("Async function definitions are not yet supported. The async def at line " + getLine(ctx)
					+ " of file " + filePath + " is unsoundly translated into a def");

		Collection<ControlFlowStructure> oldCfs = cfs;
		PyCFG newCFG = new PyCFG(buildCFGDescriptor(ctx));
		cfs = new HashSet<>();
		PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, newCFG);
		ParsedBlock r = parser.visitBlock(ctx.block());
		newCFG.getNodeList().mergeWith(r.getBody());
		newCFG.getEntrypoints().add(r.getBegin());
		addRetNodes(newCFG);
		cfs.forEach(newCFG.getDescriptor()::addControlFlowStructure);
		newCFG.simplify();
		if (currentUnit instanceof ClassUnit) {
			((ClassUnit) currentUnit).addInstanceCodeMember(newCFG);
		} else {
			currentUnit.addCodeMember(newCFG);
		}
		cfs = oldCfs;
		return newCFG;
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

	private PyParameter buildParameter(
			ParamContext param,
			Default_assignmentContext def,
			boolean first) {
		if (first && currentUnit instanceof ClassUnit)
			// the first parameter of an instance method is 'self': type it
			// with the enclosing class rather than with its annotation
			return new PyParameter(getLocation(filePath, param), param.name().getText(),
					new ReferenceType(PyClassType.register(currentUnit.getName(), (ClassUnit) currentUnit)));

		String typeHint = null;
		if (param.annotation() != null) {
			PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, currentCFG);
			typeHint = parser.visitExpression(param.annotation().expression()).toString();
		}
		Expression defaultValue = null;
		if (def != null) {
			PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, currentCFG);
			defaultValue = parser.visitExpression(def.expression());
		}
		return new PyParameter(getLocation(filePath, param), param.name().getText(), Untyped.INSTANCE, defaultValue,
				null,
				typeHint);
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

	@Override
	public ClassUnit visitClass_def_raw(
			Class_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic classes are not supported");

		Unit previous = this.currentUnit;
		String name = ctx.name().getText();
		// TODO inheritance
		ClassUnit cu = new ClassUnit(new SourceCodeLocation(name, 0, 0), program, name, true);
		PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, currentCFG);
		List<Expression> superclasses = parser.extractArguments(ctx.arguments());
		// parse anchestors
		for (Expression superclass : superclasses) {
			// if exists a class unit in the program with name
			// superclass's text: add it
			// to the anchestors
			String superClassName = imports.getOrDefault(superclass.toString(), superclass.toString());
			for (Unit programCu : this.program.getUnits())
				if (programCu instanceof CompilationUnit && programCu.getName().equals(superClassName))
					cu.addAncestor(((CompilationUnit) programCu));
		}
		if (cu.getImmediateAncestors().isEmpty() && LibrarySpecificationProvider.hierarchyRoot != null)
			cu.addAncestor(LibrarySpecificationProvider.hierarchyRoot);
		this.currentUnit = cu;
		parseClassBody(ctx.block());
		program.addUnit(cu);
		this.currentUnit = previous;
		return cu;
	}

	private void parseClassBody(
			BlockContext ctx) {
		List<Pair<VariableRef, Expression>> fields_init = new ArrayList<>();
		List<Simple_stmtContext> topLevel = new ArrayList<>();
		if (ctx.simple_stmts() != null)
			topLevel.addAll(ctx.simple_stmts().simple_stmt());
		if (ctx.statements() != null)
			for (StatementContext stmt : ctx.statements().statement()) {
				if (stmt.compound_stmt() == null)
					topLevel.addAll(stmt.simple_stmts().simple_stmt());
				else if (stmt.compound_stmt().function_def() != null) {
					// TODO decorators
					// visitFuncdef(stmt.compound_stmt().function_def());
					throw new UnsupportedStatementException("Function definitions are not yet supported inside classes");
				} else if (stmt.compound_stmt().class_def() != null) {
					// TODO decorators
					// visitClassdef(stmt.compound_stmt().class_def());
					throw new UnsupportedStatementException("Class definitions are not yet supported inside classes");
				}
			}

		for (Simple_stmtContext simple : topLevel) {
			Pair<VariableRef, Expression> p = parseField(simple);
			if (p.getLeft() != null)
				currentUnit
						.addGlobal(new Global(getLocation(filePath, ctx), currentUnit, p.getLeft().getName(), false));
			if (p.getRight() != null)
				fields_init.add(p);
		}
	}

	private Pair<VariableRef, Expression> parseField(
			Simple_stmtContext st) {
		PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, currentCFG);
		ParsedBlock simple = parser.visitSimple_stmt(st);
		Collection<Statement> nodes = simple.getBody().getNodes();
		if (nodes.size() != 1)
			throw new UnsupportedStatementException("Expected a single statement, got " + nodes.size());
		Statement result = nodes.iterator().next();
		if (result instanceof Assignment) {
			Assignment ass = (Assignment) result;
			Expression assigned = ass.getLeft();
			Expression expr = ass.getRight();
			if (assigned instanceof VariableRef)
				return Pair.of((VariableRef) assigned, expr);
		} else if (result instanceof VariableRef)
			return Pair.of((VariableRef) result, null);
		else if (result instanceof StringLiteral) // it is a comment
			return Pair.of(null, null);
		throw new UnsupportedStatementException(
				"Only variables or assignments of variable are supported as field declarations");
	}

}
