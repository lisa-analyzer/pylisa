package it.unive.pylisa;

import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.Subscript_Context;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestOrStarContext;
import it.unive.pylisa.antlr.Python3Parser.TestlistContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_compContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.antlr.v4.runtime.ParserRuleContext;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public abstract class PyFrontendBase extends Python3ParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	protected static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	protected static final Logger log = LogManager.getLogger(PyFrontend.class);

	protected Map<String, String> imports = new HashMap<>();
	/**
	 * Python program file path.
	 */
	protected final String filePath;

	boolean shouldPrependUnitAccess = true;
	protected CFG init;
	/**
	 * The LiSA program obtained from the Python program at filePath.
	 */
	protected final Program program;
	protected final PythonModuleImportManager importManager;
	/**
	 * The module currently under parsing
	 */
	protected ModuleUnit currentModule;
	protected it.unive.lisa.program.CompilationUnit objectUnit;

	/**
	 * The unit currently under parsing
	 */
	protected Unit currentUnit;
	/**
	 * Current CFG to parse
	 */
	protected PyCFG currentCFG;

	protected Collection<ControlFlowStructure> cfs;

	/**
	 * Whether or not {@link #filePath} points to a Jupyter notebook file
	 */
	protected final boolean notebook;

	/**
	 * List of the indexes of cells of a Jupyter notebook in the order they are
	 * to be executed. Only valid if {@link #notebook} is {@code true}.
	 */
	protected final List<Integer> cellOrder;

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath file path to a Python program
	 * @param notebook whether or not {@code filePath} points to a Jupyter
	 *                     notebook file
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook) {
		this(filePath, notebook, Collections.emptyList());
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder sequence of the indexes of cells of a Jupyter notebook
	 *                      in the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		this(filePath, notebook, List.of(cellOrder));
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder list of the indexes of cells of a Jupyter notebook in
	 *                      the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this.program = new Program(new PythonFeatures(), new PythonTypeSystem());
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
		this.currentModule = new ModuleUnit(new SourceCodeLocation(filePath, 0, 0),
				program, "__main__");
		this.currentUnit = currentModule;
		makeInit(program);
		this.importManager = new PythonModuleImportManager(program, init);
		program.addUnit(currentModule);
		PyModuleType.register("__main__", currentModule);
	}

	/**
	 * Returns the parsed file path.
	 *
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	protected <T> T unsupported(
			ParserRuleContext ctx,
			String description) {
		throw new UnsupportedStatementException(
				description + " at line " + getLine(ctx) + " of " + getFilePath());
	}

	protected void unsound(
			ParserRuleContext ctx,
			String description) {
		log.warn(description + " (unsound translation) at line "
				+ getLine(ctx) + " of " + getFilePath());
	}

	protected Expression makeRef(
			String name,
			CodeLocation loc) {
		if (shouldPrependUnitAccess && currentModule != null)
			return new PythonUnitAttributeAccessRef(currentCFG, loc, currentModule,
					new Global(loc, currentModule, name, false));
		return new VariableRef(currentCFG, loc, name);
	}

	@FunctionalInterface
	protected interface BinaryFactory {
		Expression make(
				PyCFG cfg,
				CodeLocation loc,
				Expression l,
				Expression r);
	}

	protected <C extends ParserRuleContext> Expression foldBinaryOp(
			List<C> operands,
			java.util.function.Function<C, Expression> sub,
			BinaryFactory factory,
			CodeLocation loc) {
		int n = operands.size();
		if (n == 1)
			return sub.apply(operands.get(0));
		Expression acc = factory.make(currentCFG, loc,
				sub.apply(operands.get(n - 2)), sub.apply(operands.get(n - 1)));
		for (int i = n - 3; i >= 0; i--)
			acc = factory.make(currentCFG, loc, sub.apply(operands.get(i)), acc);
		return acc;
	}

	protected static String transformToCode(
			List<String> code_list) {
		StringBuilder result = new StringBuilder();
		for (String s : code_list)
			result.append(s).append("\n");
		return result.toString();
	}

	protected int getLine(
			ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	protected int getCol(
			ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	public SourceCodeLocation getLocation(
			ParserRuleContext ctx) {
		return new SourceCodeLocation(this.getFilePath(), getLine(ctx), getCol(ctx));
	}

	protected CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, currentModule, false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

	protected CodeMemberDescriptor buildInitModuleCFGDesriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, currentModule, false, "$init", cfgArgs);
	}

	protected CodeMemberDescriptor buildInitClassCFGDesriptor(
			CodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, currentUnit, false, "$init", cfgArgs);
	}

	protected CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl, Unit unit) {
		String funcName = "$call";
		PyParameter[] cfgArgs = visitParameters(funcDecl.parameters());

		return new CodeMemberDescriptor(getLocation(funcDecl), unit,
				false,
				funcName, cfgArgs);
	}

	protected void addRetNodesToCurrentCFG() {
		Ret ret = new Ret(currentCFG, currentCFG.getDescriptor().getLocation());
		if (currentCFG.getNodesCount() == 0) {
			// empty method, so the ret is also the entrypoint
			currentCFG.addNode(ret, true);
		} else {
			// every non-throwing instruction that does not have a follower
			// is ending the method
			Collection<Statement> preExits = new LinkedList<>();
			for (Statement st : currentCFG.getNodes())
				if (!st.stopsExecution() && currentCFG.followersOf(st).isEmpty())
					preExits.add(st);
			if (!preExits.isEmpty()) {
				currentCFG.addNode(ret);
				for (Statement st : preExits)
					currentCFG.addEdge(new SequentialEdge(st, ret));
				for (VariableTableEntry entry : currentCFG.getDescriptor().getVariables())
					if (preExits.contains(entry.getScopeEnd()))
						entry.setScopeEnd(ret);
			}
		}
	}

	private CFG makeInit(
			Program program) {
		init = new CFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, SyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}

	protected StringLiteral strip(
			CodeLocation location,
			String string) {
		// ', ''', ", """
		if (string.startsWith("'''") && string.endsWith("'''"))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "'''");
		if (string.startsWith("\"\"\"") && string.endsWith("\"\"\""))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "\"\"\"");
		if (string.startsWith("'") && string.endsWith("'"))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "'");
		if (string.startsWith("\"") && string.endsWith("\""))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "\"");
		return new PyStringLiteral(currentCFG, location, string, "\"");
	}

	protected Boolean isADict(
			DictorsetmakerContext ctx) {
		return ctx == null || ctx.test().size() == 2 * ctx.COLON().size();
	}

	protected List<Pair<Expression, Expression>> extractPairsFromDict(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx.test().size() != 2 * ctx.COLON().size())
			throw new UnsupportedStatementException(
					"We support only initialization of dictonaries in the form of <key> : <value>");
		// it is a Dict
		for (int i = 0; i < ctx.COLON().size(); i++) {
			Expression left = visitTest(ctx.test(2 * i));
			Expression right = visitTest(ctx.test(2 * i + 1));
			result.add(Pair.of(left, right));
		}
		return result;
	}

	protected List<Expression> extractElementsFromSet(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < ctx.test().size(); i++) {
			Expression e = visitTest(ctx.test(i));
			result.add(e);
		}
		return result;
	}

	protected List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext ctx) {
		if (ctx.test() != null) {
			List<Expression> r = new ArrayList<>(1);
			r.add(visitTest(ctx.test()));
			return r;
		} else
			return visitTestlist(ctx.testlist());
	}

	protected List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : ctx.subscript_())
			result.add(visitSubscript_(e));
		return result;
	}

	protected List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.testOrStar() == null || ctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : ctx.testOrStar())
			result.add(visitTestOrStar(e));
		return result;
	}

	protected List<Expression> convertAssignmentsToByNameParameters(
			List<Expression> pars) {
		List<Expression> converted = new ArrayList<>(pars.size());
		for (Expression e : pars)
			if (!(e instanceof Assignment))
				converted.add(e);
			else
				converted.add(new NamedParameterExpression(e.getCFG(), e.getLocation(),
						((Assignment) e).getLeft().toString(), ((Assignment) e).getRight()));
		return converted;
	}

	// Abstract methods that subclasses must implement (needed by helpers above)
	public abstract Expression visitTest(
			TestContext ctx);

	public abstract Expression visitTestOrStar(
			TestOrStarContext ctx);

	public abstract Expression visitSubscript_(
			Subscript_Context ctx);

	public abstract List<Expression> visitTestlist(
			TestlistContext ctx);

	public abstract PyParameter[] visitParameters(
			ParametersContext ctx);
}
