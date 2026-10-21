package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
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
		ClassUnit signature = new ClassUnit(getLocation(filePath, ctx), program, name, false);
		Annotation bases = bases(ctx);
		signature.addAnnotation(bases);
		// the bases are defined before the class (also those imported from
		// other modules, that are parsed first)
		List<ClassUnit> ancestors = ancestors(bases);
		// the class is registered before its body is parsed, so that its
		// methods can refer to it (e.g. C.m(...) in a method of C)
		PyClassType.register(name, signature);
		program.addUnit(signature);
		if (ancestors != null) {
			// the methods of the class are those defined in its body, and the
			// ones of its bases (see PyMro)
			for (ClassUnit ancestor : ancestors)
				signature.addAncestor(ancestor);
			signature.addAnnotation(new Annotation(PyBinaryDispatch.FULLY_DEFINED));
		} else
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

	// the classes the bases of the class refer to, in order, or null if some
	// of them is not known (e.g. a class of a module that is not modeled, or
	// a base that is not a name)
	private List<ClassUnit> ancestors(
			Annotation bases) {
		List<ClassUnit> ancestors = new ArrayList<>();
		for (AnnotationMember member : bases.getAnnotationMembers()) {
			String base = member.getValue().toString();
			if (member.getId().equals(ClassBases.METACLASS)) {
				if (!base.equals("ABCMeta") && !base.equals("abc.ABCMeta"))
					// a metaclass might change the attributes of the class
					return null;
				continue;
			}
			if (base.equals("object") || base.equals("ABC") || base.equals("abc.ABC"))
				// they define no attribute that is modeled
				continue;
			ClassUnit unit = classNamed(base);
			if (unit == null)
				return null;
			ancestors.add(unit);
		}
		return ancestors;
	}

	// the class with the given name (e.g. B, m.B for a class of the local
	// module m, or the name B is imported as)
	private ClassUnit classNamed(
			String name) {
		String simple = name;
		int dot = name.lastIndexOf('.');
		if (dot >= 0) {
			if (imports.module(name.substring(0, dot)) == null)
				return null;
			simple = name.substring(dot + 1);
		} else if (imports.member(name) != null)
			simple = imports.member(name).name();
		Unit unit = program.getUnit(simple);
		return unit instanceof ClassUnit ? (ClassUnit) unit : null;
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
