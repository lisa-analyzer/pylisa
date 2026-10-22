package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.logging.TimerLogger;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.lisa.util.frontend.CFGTweaker;
import it.unive.lisa.util.frontend.ControlFlowTracker;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.statement.PyNamespacesInit;
import it.unive.pylisa.scoping.Symtable;
import it.unive.pylisa.scoping.SymtableBuilder;
import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;
import java.util.function.Function;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.io.FilenameUtils;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class PyFileParser
		extends
		PythonParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	private static final Logger log = LogManager.getLogger(PyFileParser.class);

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
	private final CodeUnit rootUnit;

	private final LocalImports imports;

	/**
	 * Builds the parser for a Python program at {@code filePath}.
	 *
	 * @param program  the LiSA program to which the parsed CFGs will be added
	 * @param filePath file path to a Python program
	 */
	public PyFileParser(
			Program program,
			String filePath) {
		this(program, filePath, LocalImports.NONE);
	}

	/**
	 * Builds the parser for a Python program at {@code filePath}.
	 *
	 * @param program  the LiSA program to which the parsed CFGs will be added
	 * @param filePath file path to a Python program
	 * @param imports  the names the imports of the file bind to local modules
	 */
	public PyFileParser(
			Program program,
			String filePath,
			LocalImports imports) {
		this.program = program;
		this.filePath = filePath;
		this.imports = imports;
		this.rootUnit = new CodeUnit(
				new SourceCodeLocation(filePath, 0, 0),
				program,
				FilenameUtils.removeExtension(filePath));
	}

	/**
	 * Returns the parsed unit. Note that the unit will be empty unless
	 * {@link #parse(ParseTree)} is called first.
	 *
	 * @return the parsed unit
	 */
	public CodeUnit getParsedUnit() {
		return rootUnit;
	}

	public CodeUnit parse(
			ParseTree tree)
			throws IOException,
			AnalysisSetupException {
		visit(tree);
		return rootUnit;
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
		CodeMemberDescriptor descriptor = buildMainCFGDescriptor(getLocation(filePath, ctx));
		NodeList<CFG, Statement, Edge> list = new NodeList<>(new SequentialEdge());
		Collection<Statement> entrypoints = new HashSet<>();
		// side effects on entrypoints and matrix will affect the cfg
		PyCFG cfg = new PyCFG(descriptor, entrypoints, list);
		ControlFlowTracker control = new ControlFlowTracker();
		Symtable symtable = SymtableBuilder.build(filePath, rootUnit.getName(), ctx);
		PyStatementParser parser = new PyStatementParser(program, filePath, rootUnit, cfg, control, imports,
				symtable);
		ParsedBlock r;
		if (ctx.statements() == null) {
			// an empty file (e.g. the __init__.py of a package)
			NoOp noop = new NoOp(cfg, getLocation(filePath, ctx));
			NodeList<CFG, Statement, Edge> block = new NodeList<>(new SequentialEdge());
			block.addNode(noop);
			r = new ParsedBlock(noop, block, noop);
		} else
			r = TimerLogger.execSupplier(log, "Parsing statement list of " + filePath,
					() -> parser.visitStatements(ctx.statements()));
		list.mergeWith(r.getBody());
		// the namespaces must be part of all the states of the module
		PyNamespacesInit init = new PyNamespacesInit(cfg,
				new DerivedLocation(getLocation(filePath, ctx), "init-namespaces"));
		list.addNode(init);
		list.addEdge(new SequentialEdge(init, r.getBegin()));
		entrypoints.add(init);
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
		rootUnit.addCodeMember(cfg);
		return cfg;
	}

	private CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, rootUnit, false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

}
