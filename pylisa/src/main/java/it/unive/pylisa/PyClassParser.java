package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.lisa.util.frontend.CFGTweaker;
import it.unive.lisa.util.frontend.ControlFlowTracker;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.antlr.PythonParser.ArgsContext;
import it.unive.pylisa.antlr.PythonParser.Assignment_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Class_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.ExpressionContext;
import it.unive.pylisa.antlr.PythonParser.Kwarg_or_double_starredContext;
import it.unive.pylisa.antlr.PythonParser.Kwarg_or_starredContext;
import it.unive.pylisa.antlr.PythonParser.Starred_expressionContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.function.Function;
import org.antlr.v4.runtime.tree.ParseTree;

public class PyClassParser
		extends
		PythonParserBaseVisitor<Object> {

	/**
	 * The name of the code member holding the body of a class.
	 */
	public static final String INSTRUMENTED_CLASS_INIT_NAME = "$class_init";

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
	// the names the imports of the file bind to local modules
	private final LocalImports imports;

	public PyClassParser(
			Program program,
			String filePath,
			LocalImports imports) {
		this.program = program;
		this.filePath = filePath;
		this.imports = imports;
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
		signature.addAnnotation(bases(ctx));
		// the class is registered before its body is parsed, so that its
		// methods can refer to it (e.g. C.m(...) in a method of C)
		PyClassType.register(name, signature);
		program.addUnit(signature);
		if (ctx.arguments() == null || ctx.arguments().getChildCount() == 0)
			// without bases, all the methods of the class are known: they are
			// those defined in its body
			signature.addAnnotation(new Annotation(PyBinaryDispatch.FULLY_DEFINED));
		else
			signature.addAnnotation(new Annotation(PyBinaryDispatch.UNTRACKED_BASES));

		// we parse the body of the class in a synthetic method that will be
		// invoked by the class definition instruction
		CodeMemberDescriptor descriptor = buildMainCFGDescriptor(getLocation(filePath, ctx), signature);
		NodeList<CFG, Statement, Edge> list = new NodeList<>(new SequentialEdge());
		Collection<Statement> entrypoints = new HashSet<>();
		// side effects on entrypoints and matrix will affect the cfg
		PyCFG cfg = new PyCFG(descriptor, entrypoints, list);
		ControlFlowTracker control = new ControlFlowTracker();
		PyStatementParser parser = new PyStatementParser(program, filePath, signature, cfg, control, imports);
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

	// the bases of the class, as written in the source (see ClassBases)
	private static Annotation bases(
			Class_def_rawContext ctx) {
		List<String> bases = new ArrayList<>();
		String metaclass = null;
		ArgsContext args = ctx.arguments() == null ? null : ctx.arguments().args();
		if (args != null) {
			for (ParseTree child : args.children)
				if (child instanceof ExpressionContext || child instanceof Assignment_expressionContext)
					bases.add(child.getText());
				else if (child instanceof Starred_expressionContext)
					bases.add(ClassBases.UNKNOWN);
			if (args.kwargs() != null)
				for (ParseTree child : args.kwargs().children)
					if (child instanceof Kwarg_or_starredContext && ((Kwarg_or_starredContext) child).name() == null
							|| child instanceof Kwarg_or_double_starredContext
									&& ((Kwarg_or_double_starredContext) child).name() == null)
						// *bases, or **kwargs
						bases.add(ClassBases.UNKNOWN);
					else if (child instanceof Kwarg_or_starredContext
							&& ((Kwarg_or_starredContext) child).name().getText().equals("metaclass"))
						metaclass = ((Kwarg_or_starredContext) child).expression().getText();
					else if (child instanceof Kwarg_or_double_starredContext
							&& ((Kwarg_or_double_starredContext) child).name().getText().equals("metaclass"))
						metaclass = ((Kwarg_or_double_starredContext) child).expression().getText();
		}
		return ClassBases.bases(bases, metaclass);
	}

	private CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc,
			ClassUnit signature) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, signature, false, INSTRUMENTED_CLASS_INIT_NAME, cfgArgs);
	}

}
