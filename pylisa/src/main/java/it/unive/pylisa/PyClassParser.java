package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.lisa.util.frontend.CFGTweaker;
import it.unive.lisa.util.frontend.ControlFlowTracker;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.antlr.PythonParser.Class_def_rawContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import java.util.Collection;
import java.util.HashSet;
import java.util.function.Function;

public class PyClassParser
		extends
		PythonParserBaseVisitor<Object> {

	private static final String INSTRUMENTED_CLASS_INIT_NAME = "$class_init";

	/**
	 * Python program file path.
	 */
	private final String filePath;

	/**
	 * The LiSA program obtained from the Python program at filePath.
	 */
	private final Program program;

	/**
	 * Builds the parser for a Python program at {@code filePath}.
	 *
	 * @param program  the LiSA program to which the parsed CFGs will be added
	 * @param filePath file path to a Python program
	 */
	public PyClassParser(
			Program program,
			String filePath) {
		this.program = program;
		this.filePath = filePath;
	}

	@Override
	public ClassUnit visitClass_def_raw(
			Class_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic classes are not supported");

		String name = ctx.name().getText();
		// we do not track inheritance here since it needs runtime symbol
		// resolution we will do that during the analysis. this should not cause
		// problems since we will do that where the signature is defined, and
		// not where it is used.
		ClassUnit signature = new ClassUnit(getLocation(filePath, ctx), program, name, false);

		// we parse the body of the class in a synthetic method that will be
		// invoked by the class definition instruction
		CodeMemberDescriptor descriptor = buildMainCFGDescriptor(getLocation(filePath, ctx), signature);
		NodeList<CFG, Statement, Edge> list = new NodeList<>(new SequentialEdge());
		Collection<Statement> entrypoints = new HashSet<>();
		// side effects on entrypoints and matrix will affect the cfg
		PyCFG cfg = new PyCFG(descriptor, entrypoints, list);
		ControlFlowTracker control = new ControlFlowTracker();
		PyStatementParser parser = new PyStatementParser(program, filePath, signature, cfg, control);
		ParsedBlock r = parser.visitBlock(ctx.block());
		list.mergeWith(r.getBody());
		entrypoints.add(r.getBegin());
		Function<String, ParsingException> factory = (
				msg) -> new ParsingException(
						"parse error",
						ParsingException.Type.PARSING_ERROR,
						msg,
						getLocation(filePath, ctx));
		// CFGTweaker.splitProtectedYields(cfg, factory);
		// CFGTweaker.addFinallyEdges(cfg, factory);
		CFGTweaker.addReturns(cfg, factory);
		cfg.simplify();
		signature.addCodeMember(cfg);
		return signature;
	}

	private CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc,
			ClassUnit signature) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, signature, false, INSTRUMENTED_CLASS_INIT_NAME, cfgArgs);
	}

}
