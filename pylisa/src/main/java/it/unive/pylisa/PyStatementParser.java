package it.unive.pylisa;

import static it.unive.pylisa.PyParsingUtils.getLine;
import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.IfThenElse;
import it.unive.lisa.program.cfg.controlFlow.Loop;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.FalseEdge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.edge.TrueEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Return;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.lisa.util.frontend.ControlFlowTracker;
import it.unive.lisa.util.frontend.ParsedBlock;
import it.unive.pylisa.annotationvalues.DecoratedAnnotation;
import it.unive.pylisa.antlr.PythonParser.Annotated_rhsContext;
import it.unive.pylisa.antlr.PythonParser.ArgumentsContext;
import it.unive.pylisa.antlr.PythonParser.Assert_stmtContext;
import it.unive.pylisa.antlr.PythonParser.AssignmentContext;
import it.unive.pylisa.antlr.PythonParser.Assignment_expressionContext;
import it.unive.pylisa.antlr.PythonParser.AtomContext;
import it.unive.pylisa.antlr.PythonParser.AugassignContext;
import it.unive.pylisa.antlr.PythonParser.Await_primaryContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_andContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_orContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_xorContext;
import it.unive.pylisa.antlr.PythonParser.BlockContext;
import it.unive.pylisa.antlr.PythonParser.Break_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Case_blockContext;
import it.unive.pylisa.antlr.PythonParser.Class_defContext;
import it.unive.pylisa.antlr.PythonParser.Closed_patternContext;
import it.unive.pylisa.antlr.PythonParser.Compare_op_bitwise_or_pairContext;
import it.unive.pylisa.antlr.PythonParser.ComparisonContext;
import it.unive.pylisa.antlr.PythonParser.Compound_stmtContext;
import it.unive.pylisa.antlr.PythonParser.ConjunctionContext;
import it.unive.pylisa.antlr.PythonParser.Continue_stmtContext;
import it.unive.pylisa.antlr.PythonParser.DecoratorsContext;
import it.unive.pylisa.antlr.PythonParser.Default_assignmentContext;
import it.unive.pylisa.antlr.PythonParser.Del_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Del_t_atomContext;
import it.unive.pylisa.antlr.PythonParser.Del_targetContext;
import it.unive.pylisa.antlr.PythonParser.DisjunctionContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_namesContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_nameContext;
import it.unive.pylisa.antlr.PythonParser.Double_starred_kvpairContext;
import it.unive.pylisa.antlr.PythonParser.Double_starred_kvpairsContext;
import it.unive.pylisa.antlr.PythonParser.Elif_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Else_blockContext;
import it.unive.pylisa.antlr.PythonParser.ExpressionContext;
import it.unive.pylisa.antlr.PythonParser.FactorContext;
import it.unive.pylisa.antlr.PythonParser.For_if_clauseContext;
import it.unive.pylisa.antlr.PythonParser.For_if_clausesContext;
import it.unive.pylisa.antlr.PythonParser.For_stmtContext;
import it.unive.pylisa.antlr.PythonParser.FstringContext;
import it.unive.pylisa.antlr.PythonParser.Fstring_middleContext;
import it.unive.pylisa.antlr.PythonParser.Fstring_replacement_fieldContext;
import it.unive.pylisa.antlr.PythonParser.Function_defContext;
import it.unive.pylisa.antlr.PythonParser.GenexpContext;
import it.unive.pylisa.antlr.PythonParser.Global_stmtContext;
import it.unive.pylisa.antlr.PythonParser.If_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Import_fromContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_targetsContext;
import it.unive.pylisa.antlr.PythonParser.Import_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_stmtContext;
import it.unive.pylisa.antlr.PythonParser.InversionContext;
import it.unive.pylisa.antlr.PythonParser.Kwarg_or_starredContext;
import it.unive.pylisa.antlr.PythonParser.KwargsContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_paramContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_param_maybe_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_param_no_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_param_with_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_parametersContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_star_etcContext;
import it.unive.pylisa.antlr.PythonParser.LambdefContext;
import it.unive.pylisa.antlr.PythonParser.ListcompContext;
import it.unive.pylisa.antlr.PythonParser.Literal_patternContext;
import it.unive.pylisa.antlr.PythonParser.Match_stmtContext;
import it.unive.pylisa.antlr.PythonParser.NameContext;
import it.unive.pylisa.antlr.PythonParser.Named_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Pass_stmtContext;
import it.unive.pylisa.antlr.PythonParser.PatternContext;
import it.unive.pylisa.antlr.PythonParser.PowerContext;
import it.unive.pylisa.antlr.PythonParser.PrimaryContext;
import it.unive.pylisa.antlr.PythonParser.Raise_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Return_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Shift_exprContext;
import it.unive.pylisa.antlr.PythonParser.Simple_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Simple_stmtsContext;
import it.unive.pylisa.antlr.PythonParser.Single_subscript_attribute_targetContext;
import it.unive.pylisa.antlr.PythonParser.Single_targetContext;
import it.unive.pylisa.antlr.PythonParser.SliceContext;
import it.unive.pylisa.antlr.PythonParser.SlicesContext;
import it.unive.pylisa.antlr.PythonParser.Star_atomContext;
import it.unive.pylisa.antlr.PythonParser.Star_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Star_expressionsContext;
import it.unive.pylisa.antlr.PythonParser.Star_named_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Star_named_expressionsContext;
import it.unive.pylisa.antlr.PythonParser.Star_targetContext;
import it.unive.pylisa.antlr.PythonParser.Star_targetsContext;
import it.unive.pylisa.antlr.PythonParser.Starred_expressionContext;
import it.unive.pylisa.antlr.PythonParser.StatementContext;
import it.unive.pylisa.antlr.PythonParser.StatementsContext;
import it.unive.pylisa.antlr.PythonParser.StringContext;
import it.unive.pylisa.antlr.PythonParser.StringsContext;
import it.unive.pylisa.antlr.PythonParser.SumContext;
import it.unive.pylisa.antlr.PythonParser.T_primaryContext;
import it.unive.pylisa.antlr.PythonParser.Target_with_star_atomContext;
import it.unive.pylisa.antlr.PythonParser.TermContext;
import it.unive.pylisa.antlr.PythonParser.Try_stmtContext;
import it.unive.pylisa.antlr.PythonParser.TupleContext;
import it.unive.pylisa.antlr.PythonParser.While_stmtContext;
import it.unive.pylisa.antlr.PythonParser.With_itemContext;
import it.unive.pylisa.antlr.PythonParser.With_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Yield_stmtContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAccessInstanceGlobal;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyAttributeAccess;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.PyBitwiseAnd;
import it.unive.pylisa.cfg.expression.PyBitwiseLeftShift;
import it.unive.pylisa.cfg.expression.PyBitwiseNot;
import it.unive.pylisa.cfg.expression.PyBitwiseOr;
import it.unive.pylisa.cfg.expression.PyBitwiseRIghtShift;
import it.unive.pylisa.cfg.expression.PyBitwiseXor;
import it.unive.pylisa.cfg.expression.PyBuiltinRef;
import it.unive.pylisa.cfg.expression.PyClassFunctionCall;
import it.unive.pylisa.cfg.expression.PyClassRef;
import it.unive.pylisa.cfg.expression.PyDivMod;
import it.unive.pylisa.cfg.expression.PyDivision;
import it.unive.pylisa.cfg.expression.PyDoubleArrayAccess;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyFunctionRef;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyInPlaceOperation;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyIteratedElement;
import it.unive.pylisa.cfg.expression.PyListComprehension;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyMethodCall;
import it.unive.pylisa.cfg.expression.PyMinMax;
import it.unive.pylisa.cfg.expression.PyModuleCall;
import it.unive.pylisa.cfg.expression.PyMultiplication;
import it.unive.pylisa.cfg.expression.PyNameCall;
import it.unive.pylisa.cfg.expression.PyNegation;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.PySingleArrayAccess;
import it.unive.pylisa.cfg.expression.PySubtraction;
import it.unive.pylisa.cfg.expression.PyTernaryOperator;
import it.unive.pylisa.cfg.expression.PyValueCall;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.StarArgument;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyBytesLiteral;
import it.unive.pylisa.cfg.expression.literal.PyFunctionLiteral;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import it.unive.pylisa.cfg.expression.literal.PyStringLiterals;
import it.unive.pylisa.cfg.expression.literal.PyTypeLiteral;
import it.unive.pylisa.cfg.expression.unary.PyAbs;
import it.unive.pylisa.cfg.expression.unary.PyLength;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.statement.PyAssert;
import it.unive.pylisa.cfg.statement.PyRaise;
import it.unive.pylisa.cfg.statement.SimpleSuperUnresolvedCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.PyBytes;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.misc.Interval;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class PyStatementParser
		extends
		PythonParserBaseVisitor<Object> {

	private static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	private static final Logger log = LogManager.getLogger(PyStatementParser.class);

	private Map<String, String> imports = new HashMap<>();

	/**
	 * The names bound to modules or other namespaces by import statements (e.g.
	 * {@code np} for {@code import numpy as np}, {@code os} for
	 * {@code import os.path}, {@code y} for {@code from x import y}): calls on
	 * their attributes ({@code np.array(...)}) are function calls, not method
	 * calls on a receiver.
	 */
	private final Set<String> namespaces = new HashSet<>();

	/**
	 * The variables of the comprehensions being parsed (innermost first),
	 * mapped to their names local to the comprehension (see
	 * {@link PyListComprehension}).
	 */
	private final Deque<Map<String, String>> comprehensions = new ArrayDeque<>();

	/**
	 * The builtin classes that are modeled as values, by their names in Python:
	 * their methods can be called on the class itself (e.g.
	 * {@code bytes.fromhex(s)} or {@code str.upper(s)}), and are looked up in
	 * the library class modeling them.
	 */
	private static final Map<String, String> BUILTIN_CLASSES = Map.of(
			"str", LibrarySpecificationProvider.STR,
			"bytes", LibrarySpecificationProvider.BYTES,
			"int", LibrarySpecificationProvider.INT,
			"float", LibrarySpecificationProvider.FLOAT);

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
	private final Unit currentUnit;

	/**
	 * Current CFG to parse
	 */
	private final PyCFG currentCFG;

	private final ControlFlowTracker control;

	// the names the imports of the file bind to local modules
	private final LocalImports localImports;

	// the number of loops enclosing the statement being parsed
	private int loops = 0;

	/**
	 * Builds the parser for a Python program at {@code filePath}.
	 *
	 * @param program     the LiSA program to which the parsed CFGs will be
	 *                        added
	 * @param filePath    file path to a Python program
	 * @param currentUnit the unit currently under parsing
	 * @param currentCFG  the CFG currently under parsing
	 * @param control     the control flow tracker to use for parsing
	 */
	public PyStatementParser(
			Program program,
			String filePath,
			Unit currentUnit,
			PyCFG currentCFG,
			ControlFlowTracker control,
			LocalImports localImports) {
		this.program = program;
		this.filePath = filePath;
		this.currentUnit = currentUnit;
		this.currentCFG = currentCFG;
		this.control = control;
		this.localImports = localImports;
	}

	@Override
	public ParsedBlock visitStatements(
			StatementsContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;
		boolean canProceed = true;
		for (StatementContext stmt : ctx.statement()) {
			if (!canProceed)
				throw new ParsingException("deadcode",
						ParsingException.Type.MALFORMED_SOURCE,
						"Instruction cannot be followed by other ones",
						getLocation(filePath, stmt));
			ParsedBlock parsed = visitStatement(stmt);
			block.mergeWith(parsed.getBody());
			if (first == null)
				first = parsed.getBegin();
			if (last != null)
				block.addEdge(new SequentialEdge(last, parsed.getBegin()));
			last = parsed.getEnd();
			canProceed = parsed.canBeContinued();
		}

		return new ParsedBlock(first, block, last);
	}

	@Override
	public ParsedBlock visitSimple_stmts(
			Simple_stmtsContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;
		boolean canProceed = true;
		for (Simple_stmtContext stmt : ctx.simple_stmt()) {
			if (!canProceed)
				throw new ParsingException("deadcode",
						ParsingException.Type.MALFORMED_SOURCE,
						"Instruction cannot be followed by other ones",
						getLocation(filePath, stmt));
			ParsedBlock parsed = visitSimple_stmt(stmt);
			block.mergeWith(parsed.getBody());
			if (first == null)
				first = parsed.getBegin();
			if (last != null)
				block.addEdge(new SequentialEdge(last, parsed.getBegin()));
			last = parsed.getEnd();
			canProceed = parsed.canBeContinued();
		}

		return new ParsedBlock(first, block, last);
	}

	@Override
	public ParsedBlock visitBlock(
			BlockContext ctx) {
		if (ctx.statements() != null)
			return visitStatements(ctx.statements());
		else
			return visitSimple_stmts(ctx.simple_stmts());
	}

	@Override
	public ParsedBlock visitStatement(
			StatementContext ctx) {
		if (ctx.compound_stmt() != null)
			return visitCompound_stmt(ctx.compound_stmt());
		else
			return visitSimple_stmts(ctx.simple_stmts());
	}

	@Override
	public ParsedBlock visitSimple_stmt(
			Simple_stmtContext ctx) {
		Statement result;
		if (ctx.assignment() != null && ctx.assignment().augassign() != null)
			return augmentedAssignment(ctx.assignment());
		else if (ctx.assignment() != null)
			result = visitAssignment(ctx.assignment());
		else if (ctx.star_expressions() != null)
			result = visitStar_expressions(ctx.star_expressions());
		else if (ctx.return_stmt() != null)
			result = visitReturn_stmt(ctx.return_stmt());
		else if (ctx.import_stmt() != null)
			result = visitImport_stmt(ctx.import_stmt());
		else if (ctx.raise_stmt() != null)
			result = visitRaise_stmt(ctx.raise_stmt());
		else if (ctx.pass_stmt() != null)
			result = visitPass_stmt(ctx.pass_stmt());
		else if (ctx.del_stmt() != null)
			result = visitDel_stmt(ctx.del_stmt());
		else if (ctx.yield_stmt() != null)
			result = visitYield_stmt(ctx.yield_stmt());
		else if (ctx.assert_stmt() != null)
			result = visitAssert_stmt(ctx.assert_stmt());
		else if (ctx.break_stmt() != null)
			result = visitBreak_stmt(ctx.break_stmt());
		else if (ctx.continue_stmt() != null)
			result = visitContinue_stmt(ctx.continue_stmt());
		else if (ctx.global_stmt() != null)
			result = visitGlobal_stmt(ctx.global_stmt());
		else if (ctx.nonlocal_stmt() != null)
			result = visitNonlocal_stmt(ctx.nonlocal_stmt());
		else if (ctx.type_alias() != null)
			throw new UnsupportedStatementException("type alias statements are not supported");
		else
			throw new UnsupportedStatementException("Simple statement not yet supported");

		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		block.addNode(result);
		return new ParsedBlock(result, block, result);
	}

	@Override
	public Statement visitAssignment(
			AssignmentContext ctx) {
		if (ctx.COLON() != null)
			throw new UnsupportedStatementException("annotated assignments are not supported");
		if (ctx.augassign() != null)
			// see augmentedAssignment
			throw new IllegalStateException("augmented assignments are translated by augmentedAssignment");

		Expression value = visitAnnotated_rhs(ctx.annotated_rhs());
		List<Star_targetsContext> targets = ctx.star_targets();
		for (int i = targets.size() - 1; i >= 0; i--)
			value = new PyAssign(currentCFG, getLocation(filePath, ctx), visitStar_targets(targets.get(i)), value);
		return value;
	}

	private Expression visitSingleTarget(
			Single_targetContext ctx) {
		if (ctx.name() != null) {
			NativeCFG builtin = PyBuiltinRef.builtin(program, ctx.name().getText());
			if (builtin != null)
				// e.g. q = len
				return new PyBuiltinRef(currentCFG, getLocation(filePath, ctx), builtin);
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		}
		if (ctx.single_target() != null)
			return visitSingleTarget(ctx.single_target());
		return visitSingleSubscriptAttributeTarget(ctx.single_subscript_attribute_target());
	}

	private Expression visitSingleSubscriptAttributeTarget(
			Single_subscript_attribute_targetContext ctx) {
		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return attribute(base, ctx.name().getText(), getLocation(filePath, ctx));

		List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
		if (indexes.size() == 1)
			return new PySingleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
					indexes.get(0));
		else if (indexes.size() == 2)
			return new PyDoubleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
					indexes.get(0),
					indexes.get(1));
		throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
	}

	/**
	 * Translates {@code target op= value}: the base and the index of the target
	 * (e.g. {@code a} and {@code i} in {@code a[i] += v}) are evaluated once,
	 * into fresh variables, then the target is read, the value is evaluated,
	 * and the result of {@code op} is written to the target (see
	 * {@link #buildAugmentedOp}). The target is read and written through
	 * distinct expressions, with distinct locations, as each node of the cfg
	 * must be distinct from the others.
	 */
	private ParsedBlock augmentedAssignment(
			AssignmentContext ctx) {
		SourceCodeLocation loc = getLocation(filePath, ctx);
		String tag = loc.getLine() + ":" + loc.getCol();
		Single_targetContext target = ctx.single_target();
		while (target.single_target() != null)
			// (x) += 1
			target = target.single_target();

		List<Statement> prelude = new ArrayList<>();
		Expression write, read;
		if (target.name() != null) {
			String name = target.name().getText();
			write = new VariableRef(currentCFG, getLocation(filePath, target), name);
			read = new VariableRef(currentCFG, new DerivedLocation(getLocation(filePath, target), "read"), name);
		} else {
			Single_subscript_attribute_targetContext access = target.single_subscript_attribute_target();
			String base = "$aug@" + tag;
			prelude.add(new PyAssign(currentCFG, new DerivedLocation(loc, "base"),
					new VariableRef(currentCFG, new DerivedLocation(loc, "base-target"), base),
					visitTPrimary(access.t_primary())));
			Expression writeBase = new VariableRef(currentCFG, new DerivedLocation(loc, "base-write"), base);
			Expression readBase = new VariableRef(currentCFG, new DerivedLocation(loc, "base-read"), base);
			if (access.DOT() != null) {
				String name = access.name().getText();
				write = attribute(writeBase, name, new DerivedLocation(loc, "write"));
				read = attribute(readBase, name, new DerivedLocation(loc, "read"));
			} else {
				List<Expression> indexes = extractExpressionsFromSlices(access.slices());
				if (indexes.size() != 1)
					throw new UnsupportedStatementException(
							"augmented assignments to subscripts with many indexes are not supported");
				String index = "$augindex@" + tag;
				prelude.add(new PyAssign(currentCFG, new DerivedLocation(loc, "index"),
						new VariableRef(currentCFG, new DerivedLocation(loc, "index-target"), index),
						indexes.get(0)));
				write = new PySingleArrayAccess(currentCFG, new DerivedLocation(loc, "write"), Untyped.INSTANCE,
						writeBase, new VariableRef(currentCFG, new DerivedLocation(loc, "index-write"), index));
				read = new PySingleArrayAccess(currentCFG, new DerivedLocation(loc, "read"), Untyped.INSTANCE,
						readBase, new VariableRef(currentCFG, new DerivedLocation(loc, "index-read"), index));
			}
		}

		Expression rhs = visitAnnotated_rhs(ctx.annotated_rhs());
		Statement assignment = new PyAssign(currentCFG, loc, write, buildAugmentedOp(ctx.augassign(), loc, read, rhs));
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement previous = null;
		for (Statement st : prelude) {
			block.addNode(st);
			if (previous != null)
				block.addEdge(new SequentialEdge(previous, st));
			previous = st;
		}
		block.addNode(assignment);
		if (previous != null)
			block.addEdge(new SequentialEdge(previous, assignment));
		return new ParsedBlock(prelude.isEmpty() ? assignment : prelude.get(0), block, assignment);
	}

	private Expression buildAugmentedOp(
			AugassignContext ctx,
			SourceCodeLocation loc,
			Expression left,
			Expression right) {
		if (ctx.ATEQUAL() != null)
			return new PyMatMul(currentCFG, loc, left, right);

		// x op= y calls __iop__, falling back to __op__ and __rop__
		String symbol, name;
		if (ctx.PLUSEQUAL() != null) {
			symbol = "+=";
			name = "add";
		} else if (ctx.MINEQUAL() != null) {
			symbol = "-=";
			name = "sub";
		} else if (ctx.STAREQUAL() != null) {
			symbol = "*=";
			name = "mul";
		} else if (ctx.SLASHEQUAL() != null) {
			symbol = "/=";
			name = "truediv";
		} else if (ctx.PERCENTEQUAL() != null) {
			symbol = "%=";
			name = "mod";
		} else if (ctx.AMPEREQUAL() != null) {
			symbol = "&=";
			name = "and";
		} else if (ctx.VBAREQUAL() != null) {
			symbol = "|=";
			name = "or";
		} else if (ctx.CIRCUMFLEXEQUAL() != null) {
			symbol = "^=";
			name = "xor";
		} else if (ctx.LEFTSHIFTEQUAL() != null) {
			symbol = "<<=";
			name = "lshift";
		} else if (ctx.RIGHTSHIFTEQUAL() != null) {
			symbol = ">>=";
			name = "rshift";
		} else if (ctx.DOUBLESTAREQUAL() != null) {
			symbol = "**=";
			name = "pow";
		} else if (ctx.DOUBLESLASHEQUAL() != null) {
			symbol = "//=";
			name = "floordiv";
		} else
			throw new UnsupportedStatementException("Unknown augmented assignment operator");
		return new PyInPlaceOperation(currentCFG, loc, symbol, "__i" + name + "__", "__" + name + "__",
				"__r" + name + "__", left, right);
	}

	@Override
	public Expression visitAnnotated_rhs(
			Annotated_rhsContext ctx) {
		if (ctx.yield_expr() != null)
			throw new UnsupportedStatementException("yield expressions are not supported");
		return visitStar_expressions(ctx.star_expressions());
	}

	@Override
	public Expression visitStar_expressions(
			Star_expressionsContext ctx) {
		if (ctx.star_expression().size() == 1)
			return visitStar_expression(ctx.star_expression(0));
		List<Expression> elements = new ArrayList<>();
		for (Star_expressionContext e : ctx.star_expression())
			elements.add(visitStar_expression(e));
		return new TupleCreation(currentCFG, getLocation(filePath, ctx), elements.toArray(Expression[]::new));
	}

	@Override
	public Expression visitStar_expression(
			Star_expressionContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(filePath, ctx), visitBitwise_or(ctx.bitwise_or()));
		return visitExpression(ctx.expression());
	}

	@Override
	public Expression visitStar_targets(
			Star_targetsContext ctx) {
		if (ctx.star_target().size() == 1)
			return visitTarget(ctx.star_target(0));
		List<Expression> elements = new ArrayList<>();
		for (Star_targetContext t : ctx.star_target())
			elements.add(visitTarget(t));
		return new TupleCreation(currentCFG, getLocation(filePath, ctx), elements.toArray(Expression[]::new));
	}

	private Expression visitTarget(
			Star_targetContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(filePath, ctx), visitTarget(ctx.star_target()));
		return visitTargetWithStarAtom(ctx.target_with_star_atom());
	}

	private Expression visitTargetWithStarAtom(
			Target_with_star_atomContext ctx) {
		if (ctx.star_atom() != null)
			return visitStarAtom(ctx.star_atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return attribute(base, ctx.name().getText(), getLocation(filePath, ctx));

		List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
		if (indexes.size() == 1)
			return new PySingleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
					indexes.get(0));
		else if (indexes.size() == 2)
			return new PyDoubleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
					indexes.get(0),
					indexes.get(1));
		throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
	}

	private Expression visitStarAtom(
			Star_atomContext ctx) {
		if (ctx.name() != null) {
			NativeCFG builtin = PyBuiltinRef.builtin(program, ctx.name().getText());
			if (builtin != null)
				// e.g. q = len
				return new PyBuiltinRef(currentCFG, getLocation(filePath, ctx), builtin);
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		}
		if (ctx.target_with_star_atom() != null)
			return visitTargetWithStarAtom(ctx.target_with_star_atom());
		// a nested target, e.g. (b, c) in a, (b, c) = ...
		List<Star_targetContext> targets = ctx.star_targets_tuple_seq() != null
				? ctx.star_targets_tuple_seq().star_target()
				: ctx.star_targets_list_seq() != null ? ctx.star_targets_list_seq().star_target() : List.of();
		if (targets.isEmpty())
			throw new UnsupportedStatementException("Empty unpacking targets are not supported");
		List<Expression> elements = new ArrayList<>();
		for (Star_targetContext t : targets)
			elements.add(visitTarget(t));
		return new TupleCreation(currentCFG, getLocation(filePath, ctx), elements.toArray(Expression[]::new));
	}

	private Expression visitTPrimary(
			T_primaryContext ctx) {
		if (ctx.t_primary() == null)
			return visitAtom(ctx.atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return attribute(base, ctx.name().getText(), getLocation(filePath, ctx));
		else if (ctx.LSQB() != null) {
			List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
			if (indexes.size() == 1)
				return new PySingleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
						indexes.get(0));
			else if (indexes.size() == 2)
				return new PyDoubleArrayAccess(currentCFG, getLocation(filePath, ctx), Untyped.INSTANCE, base,
						indexes.get(0), indexes.get(1));
			throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
		} else
			throw new UnsupportedStatementException(
					"Call/generator expressions are not supported as assignment targets");
	}

	@Override
	public Statement visitDel_stmt(
			Del_stmtContext ctx) {
		List<Expression> targets = new ArrayList<>();
		for (Del_targetContext t : ctx.del_targets().del_target())
			targets.add(visitDelTarget(t));

		return new UnresolvedCall(
				currentCFG,
				getLocation(filePath, ctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"del",
				LeftToRightEvaluation.INSTANCE,
				targets.toArray(Expression[]::new));
	}

	private Expression visitDelTarget(
			Del_targetContext ctx) {
		if (ctx.del_t_atom() != null)
			return visitDelTAtom(ctx.del_t_atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return attribute(base, ctx.name().getText(), getLocation(filePath, ctx));

		// del x[i] calls x.__delitem__(i) (e.g. removing an element of a
		// list)
		List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
		if (indexes.size() > 2)
			throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
		indexes.add(0, base);
		return new PyMethodCall(currentCFG, getLocation(filePath, ctx), "__delitem__", LeftToRightEvaluation.INSTANCE,
				indexes.toArray(Expression[]::new));
	}

	private Expression visitDelTAtom(
			Del_t_atomContext ctx) {
		if (ctx.name() != null) {
			NativeCFG builtin = PyBuiltinRef.builtin(program, ctx.name().getText());
			if (builtin != null)
				// e.g. q = len
				return new PyBuiltinRef(currentCFG, getLocation(filePath, ctx), builtin);
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		}
		if (ctx.del_target() != null)
			return visitDelTarget(ctx.del_target());
		throw new UnsupportedStatementException("Tuple/list del targets are not supported");
	}

	@Override
	public Statement visitPass_stmt(
			Pass_stmtContext ctx) {
		return new NoOp(currentCFG, getLocation(filePath, ctx));
	}

	@Override
	public Statement visitBreak_stmt(
			Break_stmtContext ctx) {
		Break br = new Break(currentCFG, getLocation(filePath, ctx));
		control.addModifier(br);
		return br;
	}

	@Override
	public Statement visitContinue_stmt(
			Continue_stmtContext ctx) {
		Continue cont = new Continue(currentCFG, getLocation(filePath, ctx));
		control.addModifier(cont);
		return cont;
	}

	@Override
	public Statement visitReturn_stmt(
			Return_stmtContext ctx) {
		if (ctx.star_expressions() == null)
			// a bare return returns None
			return new Return(currentCFG, getLocation(filePath, ctx),
					new PyNoneLiteral(currentCFG, getLocation(filePath, ctx)));
		return new Return(currentCFG, getLocation(filePath, ctx), visitStar_expressions(ctx.star_expressions()));
	}

	@Override
	public Statement visitYield_stmt(
			Yield_stmtContext ctx) {
		// calling a function containing yield returns a generator, without
		// executing its body: analyzing it as a plain function is unsound
		// (e.g. a generator looping forever would never return)
		throw new UnsupportedStatementException("generator functions (yield) are not supported");
	}

	@Override
	public Statement visitRaise_stmt(
			Raise_stmtContext ctx) {
		if (ctx.expression().isEmpty())
			// the exception being handled is not tracked, as except blocks
			// are not
			throw new UnsupportedStatementException("re-raising (raise without an exception) is not supported");
		SourceCodeLocation loc = getLocation(filePath, ctx);
		Expression exception = visitExpression(ctx.expression(0));
		if (exception instanceof VariableRef) {
			// raise C, with C a class, raises C()
			Unit unit = program.getUnit(((VariableRef) exception).getName());
			if (unit instanceof ClassUnit)
				exception = new PyNewObj(currentCFG, loc, "__init__",
						PyClassType.register(unit.getName(), (ClassUnit) unit));
		}
		Expression cause = ctx.expression().size() > 1 ? visitExpression(ctx.expression(1)) : null;
		return new PyRaise(currentCFG, loc, exception, cause);
	}

	@Override
	public Statement visitImport_stmt(
			Import_stmtContext ctx) {
		if (ctx.import_from() != null)
			return visitImport_from(ctx.import_from());
		else
			return visitImport_name(ctx.import_name());
	}

	@Override
	public Statement visitImport_from(
			Import_fromContext ctx) {
		String name;
		if (ctx.dotted_name() != null)
			name = dottedNameToString(ctx.dotted_name());
		else
			name = ".";

		Import_from_targetsContext targets = ctx.import_from_targets();
		if (targets.STAR() != null)
			return new FromImport(program, name, Map.of("*", "*"), currentCFG, getLocation(filePath, ctx));

		Map<String, String> components = new HashMap<>();
		for (Import_from_as_nameContext single : targets.import_from_as_names().import_from_as_name()) {
			String importedComponent = single.name(0).getText();
			String as = single.name().size() == 2 ? single.name(1).getText() : null;
			components.put(importedComponent, as);
			imports.put(importedComponent, name + "." + importedComponent);
			namespaces.add(as != null ? as : importedComponent);
		}
		return new FromImport(program, name, components, currentCFG, getLocation(filePath, ctx));
	}

	@Override
	public Statement visitImport_name(
			Import_nameContext ctx) {
		Map<String, String> libs = new HashMap<>();
		for (Dotted_as_nameContext single : ctx.dotted_as_names().dotted_as_name()) {
			String importedLibrary = dottedNameToString(single.dotted_name());
			String as = single.name() != null ? single.name().getText() : null;
			libs.put(importedLibrary, as);
			// "import a.b.c" binds "a", "import a.b.c as d" binds "d"
			namespaces.add(as != null ? as : importedLibrary.split("\\.")[0]);
		}
		return new Import(program, libs, currentCFG, getLocation(filePath, ctx));
	}

	private String dottedNameToString(
			Dotted_nameContext dotted_name) {
		if (dotted_name.dotted_name() == null)
			return dotted_name.name().getText();
		return dottedNameToString(dotted_name.dotted_name()) + "." + dotted_name.name().getText();
	}

	@Override
	public Statement visitDotted_as_name(
			Dotted_as_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitDotted_as_names(
			Dotted_as_namesContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitDotted_name(
			Dotted_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitGlobal_stmt(
			Global_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitNonlocal_stmt(
			Nonlocal_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitAssert_stmt(
			Assert_stmtContext ctx) {
		List<ExpressionContext> exprs = ctx.expression();
		return new PyAssert(
				currentCFG,
				getLocation(filePath, ctx),
				visitExpression(exprs.get(0)),
				exprs.size() > 1 ? visitExpression(exprs.get(1)) : null);
	}

	/**
	 * Translates adjacent string literals, some of which are f-strings, into
	 * the concatenation of their parts: {@code f"Hello, {name}!"} is
	 * {@code "Hello, " + str(name) + "!"}. A replacement field {@code {x!r}} is
	 * {@code repr(x)}, and {@code {x=}} is {@code "x=" + repr(x)}; format
	 * specifications ({@code {x:>5}}) and the {@code !a} conversion are not
	 * supported.
	 */
	private Expression visitFormattedStrings(
			StringsContext ctx) {
		SourceCodeLocation loc = getLocation(filePath, ctx);
		List<Expression> parts = new ArrayList<>();
		StringBuilder text = new StringBuilder();
		for (int i = 0; i < ctx.getChildCount(); i++) {
			ParseTree child = ctx.getChild(i);
			if (child instanceof StringContext) {
				String literal = child.getText();
				if (PyStringLiterals.isBytes(literal))
					throw new UnsupportedStatementException(
							"cannot mix bytes and nonbytes literals (at " + loc + ")");
				text.append(PyStringLiterals.decode(literal));
				continue;
			}

			FstringContext fstring = (FstringContext) child;
			boolean raw = fstring.FSTRING_START().getText().toLowerCase().contains("r");
			for (Fstring_middleContext middle : fstring.fstring_middle()) {
				if (middle.FSTRING_MIDDLE() != null) {
					text.append(PyStringLiterals.decodeFStringText(middle.FSTRING_MIDDLE().getText(), raw));
					continue;
				}

				Fstring_replacement_fieldContext field = middle.fstring_replacement_field();
				if (field.fstring_full_format_spec() != null
						&& !field.fstring_full_format_spec().fstring_format_spec().isEmpty())
					throw new UnsupportedStatementException("format specifications in f-strings are not supported");
				String conversion = field.fstring_conversion() == null ? null
						: field.fstring_conversion().name().getText();
				int equal = -1;
				for (int k = 0; k < field.getChildCount(); k++)
					if (field.getChild(k) instanceof TerminalNode && field.getChild(k).getText().equals("="))
						equal = k;
				if (equal >= 0) {
					// {x=} is the text of the expression up to what follows the
					// = (whitespace included), followed by repr(x) (unless
					// another conversion is given)
					ParseTree next = field.getChild(equal + 1);
					int end = (next instanceof TerminalNode ? ((TerminalNode) next).getSymbol()
							: ((ParserRuleContext) next).start).getStartIndex() - 1;
					text.append(field.start.getInputStream()
							.getText(Interval.of(field.start.getStopIndex() + 1, end)));
					if (conversion == null)
						conversion = "r";
				}
				if (conversion != null && !conversion.equals("r") && !conversion.equals("s"))
					throw new UnsupportedStatementException("the !" + conversion + " conversion is not supported");

				if (text.length() > 0)
					parts.add(new PyStringLiteral(currentCFG, loc, text.toString(), "\""));
				text.setLength(0);
				Expression value = visitAnnotated_rhs(field.annotated_rhs());
				parts.add(new UnresolvedCall(currentCFG, loc, CallType.STATIC, null,
						"r".equals(conversion) ? "repr" : "str", LeftToRightEvaluation.INSTANCE, value));
			}
		}
		if (text.length() > 0 || parts.isEmpty())
			parts.add(new PyStringLiteral(currentCFG, loc, text.toString(), "\""));

		Expression result = parts.get(0);
		for (int i = 1; i < parts.size(); i++)
			result = new PyAddition(currentCFG, loc, result, parts.get(i));
		return result;
	}

	@Override
	public ParsedBlock visitCompound_stmt(
			Compound_stmtContext ctx) {
		if (ctx.function_def() != null) {
			PyCFG fun = visitFunction_def(ctx.function_def());
			if (currentUnit instanceof ClassUnit) {
				((ClassUnit) currentUnit).addInstanceCodeMember(fun);
				// methods are registered as code members of the class
				return noOpBlock(ctx);
			}
			currentUnit.addCodeMember(fun);
			if (loops == 0 && currentCFG.getDescriptor().getName().equals(PyFileParser.INSTRUMENTED_MAIN_FUNCTION_NAME))
				// the def is executed once: there is a single function object
				// for it
				fun.getDescriptor().addAnnotation(new Annotation(PyBinaryDispatch.DEFINED_ONCE));
			// def f(...) binds f to the function, that can then be assigned to
			// other variables and called through them
			SourceCodeLocation loc = getLocation(filePath, ctx);
			PyAssign bind = new PyAssign(currentCFG, loc,
					new VariableRef(currentCFG, loc, fun.getDescriptor().getName()),
					new PyFunctionLiteral(currentCFG, loc, fun));
			NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
			block.addNode(bind);
			return new ParsedBlock(bind, block, bind);
		} else if (ctx.class_def() != null) {
			if (currentUnit instanceof ClassUnit)
				// the nested class would not be an attribute of the outer one
				// (e.g. self.Nested(...) would raise AttributeError)
				throw new UnsupportedStatementException("nested classes are not supported");
			ClassUnit cu = visitClass_def(ctx.class_def());
			PyClassType.register(cu.getName(), cu);
			program.addUnit(cu);
			// same as above: the class is registered as a unit, not inlined
			return noOpBlock(ctx);
		} else if (ctx.if_stmt() != null)
			return this.visitIf_stmt(ctx.if_stmt());
		else if (ctx.while_stmt() != null)
			return this.visitWhile_stmt(ctx.while_stmt());
		else if (ctx.for_stmt() != null)
			return this.visitFor_stmt(ctx.for_stmt());
		else if (ctx.try_stmt() != null)
			return this.visitTry_stmt(ctx.try_stmt());
		else if (ctx.with_stmt() != null)
			return this.visitWith_stmt(ctx.with_stmt());
		else if (ctx.match_stmt() != null)
			return this.visitMatch_stmt(ctx.match_stmt());
		throw new UnsupportedStatementException("Statement " + ctx + " not yet supported");
	}

	/**
	 * Builds a {@link ParsedBlock} made of a single {@link NoOp}, for compound
	 * statements (function and class definitions) that only have side effects
	 * on the enclosing unit/CFG and do not themselves execute anything.
	 */
	private ParsedBlock noOpBlock(
			Compound_stmtContext ctx) {
		NoOp noop = new NoOp(currentCFG, getLocation(filePath, ctx));
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		block.addNode(noop);
		return new ParsedBlock(noop, block, noop);
	}

	@Override
	public ParsedBlock visitIf_stmt(
			If_stmtContext ctx) {
		// flatten the (possibly nested) elif chain into a plain list of
		// (guard, block) clauses, plus a trailing else block, if any
		List<Pair<Named_expressionContext, BlockContext>> clauses = new ArrayList<>();
		clauses.add(Pair.of(ctx.named_expression(), ctx.block()));
		Elif_stmtContext elif = ctx.elif_stmt();
		Else_blockContext elseBlock = ctx.else_block();
		while (elif != null) {
			clauses.add(Pair.of(elif.named_expression(), elif.block()));
			elseBlock = elif.else_block();
			elif = elif.elif_stmt();
		}

		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement booleanGuard = visitNamed_expression(clauses.get(0).getLeft());
		block.addNode(booleanGuard);

		// Created if exit node
		NoOp ifExitNode = new NoOp(currentCFG, getLocation(filePath, ctx));
		block.addNode(ifExitNode);

		// Visit if true block
		ParsedBlock trueBlock = visitBlock(clauses.get(0).getRight());
		block.mergeWith(trueBlock.getBody());
		Statement trueEntry = trueBlock.getBegin();
		Statement trueExit = trueBlock.getEnd();

		block.addEdge(new TrueEdge(booleanGuard, trueEntry));
		if (!trueExit.stopsExecution() && !(trueExit instanceof Continue) && !(trueExit instanceof Break))
			block.addEdge(new SequentialEdge(trueExit, ifExitNode));

		List<Pair<Statement, Collection<Statement>>> branches = new LinkedList<>();
		Statement lastElifGuard = booleanGuard;
		// if clauses.size() is >1 the context contains elif
		for (int i = 1; i < clauses.size(); i++) {
			Statement elifGuard = visitNamed_expression(clauses.get(i).getLeft());
			block.addNode(elifGuard);
			block.addEdge(new FalseEdge(lastElifGuard, elifGuard));
			lastElifGuard = elifGuard;
			ParsedBlock elifBlock = visitBlock(clauses.get(i).getRight());
			block.mergeWith(elifBlock.getBody());
			branches.add(Pair.of(elifGuard, elifBlock.getBody().getNodes()));
			Statement elifEntry = elifBlock.getBegin();
			Statement elifExit = elifBlock.getEnd();

			block.addEdge(new TrueEdge(elifGuard, elifEntry));
			if (!elifExit.stopsExecution() && !(elifExit instanceof Continue) && !(elifExit instanceof Break))
				block.addEdge(new SequentialEdge(elifExit, ifExitNode));
		}

		// If statement with else
		Collection<Statement> falseStatements = new HashSet<>();
		if (elseBlock != null) {
			ParsedBlock falseBlock = visitBlock(elseBlock.block());
			block.mergeWith(falseBlock.getBody());
			falseStatements.addAll(falseBlock.getBody().getNodes());
			Statement falseEntry = falseBlock.getBegin();
			Statement falseExit = falseBlock.getEnd();

			block.addEdge(new FalseEdge(lastElifGuard, falseEntry));
			if (!falseExit.stopsExecution() && !(falseExit instanceof Continue) && !(falseExit instanceof Break))
				block.addEdge(new SequentialEdge(falseExit, ifExitNode));
		} else {
			// If statement with no else
			if (!lastElifGuard.stopsExecution() && !(lastElifGuard instanceof Continue)
					&& !(lastElifGuard instanceof Break))
				block.addEdge(new FalseEdge(lastElifGuard, ifExitNode));
		}

		for (int k = branches.size() - 1; k >= 0; k--) {
			Pair<Statement, Collection<Statement>> branch = branches.get(k);
			currentCFG.getDescriptor().addControlFlowStructure(new IfThenElse(
					currentCFG.getNodeList(),
					branch.getLeft(),
					ifExitNode,
					branch.getRight(),
					new HashSet<>(falseStatements)));
		}
		currentCFG.getDescriptor().addControlFlowStructure(new IfThenElse(
				currentCFG.getNodeList(),
				booleanGuard,
				ifExitNode,
				trueBlock.getBody().getNodes(),
				falseStatements));
		return new ParsedBlock(booleanGuard, block, ifExitNode);
	}

	/**
	 * Translates {@code match subject: case p1: ... case p2 if g: ...}: the
	 * subject is evaluated once into a fresh variable, and the cases are tested
	 * in order, as in an {@code if}/{@code elif} chain. The supported patterns
	 * are literals ({@code 1}, {@code "s"}, {@code -2}, compared with
	 * {@code ==}, and {@code None}, {@code True}, {@code False}, compared with
	 * {@code is}), dotted names ({@code Color.RED}, compared with {@code ==}),
	 * their alternatives ({@code 1 | 2}), captures ({@code case x}, binding
	 * {@code x} to the subject) and the wildcard {@code _}, that always match,
	 * and guards ({@code case p if g}, evaluated after the captures).
	 */
	@Override
	public ParsedBlock visitMatch_stmt(
			Match_stmtContext ctx) {
		if (ctx.subject_expr().named_expression() == null)
			throw new UnsupportedStatementException("match statements on tuples are not supported");
		SourceCodeLocation loc = getLocation(filePath, ctx);
		String subject = "$match@" + loc.getLine() + ":" + loc.getCol();
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		PyAssign evaluation = new PyAssign(currentCFG, loc, new VariableRef(currentCFG, loc, subject),
				visitNamed_expression(ctx.subject_expr().named_expression()));
		block.addNode(evaluation);
		NoOp exit = new NoOp(currentCFG, loc);
		block.addNode(exit);

		// the statement leading to the next case: the test of the previous
		// case when it fails (a condition), or the evaluation of the subject
		Statement previous = evaluation;
		boolean previousIsCondition = false;
		List<Pair<Statement, Collection<Statement>>> conditions = new ArrayList<>();
		List<Collection<Statement>> cases = new ArrayList<>();
		List<Case_blockContext> blocks = ctx.case_block();
		for (int i = 0; i < blocks.size(); i++) {
			Case_blockContext c = blocks.get(i);
			SourceCodeLocation cloc = getLocation(filePath, c);
			if (c.patterns().pattern() == null)
				throw new UnsupportedStatementException("sequence patterns are not supported");
			Collection<Statement> nodes = new HashSet<>();

			// a capture binds the subject before the guard is evaluated
			String capture = capture(c.patterns().pattern());
			Statement bind = null;
			if (capture != null && !capture.equals("_")) {
				bind = new PyAssign(currentCFG, cloc, new VariableRef(currentCFG, cloc, capture),
						new VariableRef(currentCFG, cloc, subject));
				block.addNode(bind);
				nodes.add(bind);
			}
			Expression test = capture != null ? null : test(c.patterns().pattern(), subject);
			if (c.guard() != null) {
				Expression guard = visitNamed_expression(c.guard().named_expression());
				test = test == null ? guard : new PyAnd(currentCFG, cloc, test, guard);
			}
			if (test != null) {
				block.addNode(test);
				nodes.add(test);
			}

			ParsedBlock body = visitBlock(c.block());
			block.mergeWith(body.getBody());
			nodes.addAll(body.getBody().getNodes());
			cases.add(nodes);
			Statement entry = bind != null ? bind : test != null ? test : body.getBegin();
			block.addEdge(previousIsCondition ? new FalseEdge(previous, entry) : new SequentialEdge(previous, entry));
			if (bind != null)
				block.addEdge(new SequentialEdge(bind, test != null ? test : body.getBegin()));
			if (test != null) {
				block.addEdge(new TrueEdge(test, body.getBegin()));
				conditions.add(Pair.of(test, body.getBody().getNodes()));
			}
			Statement end = body.getEnd();
			if (!end.stopsExecution() && !(end instanceof Continue) && !(end instanceof Break))
				block.addEdge(new SequentialEdge(end, exit));

			if (test == null) {
				// an irrefutable pattern: python rejects the cases after it
				if (i != blocks.size() - 1)
					throw new UnsupportedStatementException("irrefutable patterns must be the last ones");
				previous = null;
			} else {
				previous = test;
				previousIsCondition = true;
			}
		}
		if (previous != null)
			// no case matches
			block.addEdge(new FalseEdge(previous, exit));

		for (int k = conditions.size() - 1; k >= 0; k--) {
			// the cases after this one are executed when its test fails
			Collection<Statement> otherwise = new HashSet<>();
			for (int j = k + 1; j < cases.size(); j++)
				otherwise.addAll(cases.get(j));
			currentCFG.getDescriptor().addControlFlowStructure(new IfThenElse(currentCFG.getNodeList(),
					conditions.get(k).getLeft(), exit, conditions.get(k).getRight(), otherwise));
		}
		return new ParsedBlock(evaluation, block, exit);
	}

	// the name bound by the pattern if it is a capture (or "_" if it is the
	// wildcard), that always matches, or null
	private String capture(
			PatternContext pattern) {
		if (pattern.as_pattern() != null || pattern.or_pattern().closed_pattern().size() != 1)
			return null;
		Closed_patternContext closed = pattern.or_pattern().closed_pattern(0);
		if (closed.capture_pattern() != null)
			return closed.capture_pattern().getText();
		if (closed.wildcard_pattern() != null)
			return "_";
		if (closed.group_pattern() != null)
			return capture(closed.group_pattern().pattern());
		return null;
	}

	// the condition under which the subject matches the pattern
	private Expression test(
			PatternContext pattern,
			String subject) {
		if (pattern.as_pattern() != null)
			throw new UnsupportedStatementException("as patterns are not supported");
		Expression result = null;
		for (Closed_patternContext closed : pattern.or_pattern().closed_pattern()) {
			Expression test = test(closed, subject);
			result = result == null ? test : new PyOr(currentCFG, getLocation(filePath, closed), result, test);
		}
		return result;
	}

	private Expression test(
			Closed_patternContext pattern,
			String subject) {
		SourceCodeLocation loc = getLocation(filePath, pattern);
		VariableRef value = new VariableRef(currentCFG, loc, subject);
		if (pattern.literal_pattern() != null) {
			Literal_patternContext literal = pattern.literal_pattern();
			if (literal.NONE() != null)
				return new PyIs(currentCFG, loc, value, new PyNoneLiteral(currentCFG, loc));
			if (literal.TRUE() != null)
				return new PyIs(currentCFG, loc, value, new TrueLiteral(currentCFG, loc));
			if (literal.FALSE() != null)
				return new PyIs(currentCFG, loc, value, new FalseLiteral(currentCFG, loc));
			if (literal.complex_number() != null)
				throw new UnsupportedStatementException("complex numbers are not supported");
			Expression constant;
			if (literal.strings() != null)
				constant = strings(literal.strings(), literal);
			else {
				constant = number(literal.signed_number().NUMBER().getText(), literal);
				if (literal.signed_number().MINUS() != null)
					constant = new PyNegation(currentCFG, loc, constant);
			}
			return new PyEquals(currentCFG, loc, value, constant);
		}
		if (pattern.value_pattern() != null) {
			List<NameContext> names = pattern.value_pattern().attr().name();
			Expression constant = name(names.get(0).getText(), names.get(0));
			for (int i = 1; i < names.size(); i++)
				constant = attribute(constant, names.get(i).getText(), getLocation(filePath, names.get(i)));
			return new PyEquals(currentCFG, loc, value, constant);
		}
		if (pattern.group_pattern() != null)
			return test(pattern.group_pattern().pattern(), subject);
		if (pattern.capture_pattern() != null || pattern.wildcard_pattern() != null)
			throw new UnsupportedStatementException("captures inside alternative patterns are not supported");
		throw new UnsupportedStatementException("sequence, mapping and class patterns are not supported");
	}

	// parses the body of a loop
	private ParsedBlock visitLoopBody(
			BlockContext block) {
		loops++;
		try {
			return visitBlock(block);
		} finally {
			loops--;
		}
	}

	@Override
	public ParsedBlock visitWhile_stmt(
			While_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of while
		NoOp whileExitNode = new NoOp(currentCFG, getLocation(filePath, ctx));
		block.addNode(whileExitNode);

		Statement condition = visitNamed_expression(ctx.named_expression());
		block.addNode(condition);

		ParsedBlock trueBlock = visitLoopBody(ctx.block());

		block.mergeWith(trueBlock.getBody());
		block.addEdge(new TrueEdge(condition, trueBlock.getBegin()));
		block.addEdge(new SequentialEdge(trueBlock.getEnd(), condition));

		// check if there's an else condition for the while
		Statement firstFollower;
		if (ctx.else_block() != null) {
			ParsedBlock falseBlock = visitBlock(
					ctx.else_block().block());
			block.mergeWith(falseBlock.getBody());
			block.addEdge(new FalseEdge(condition, falseBlock.getBegin()));
			block.addEdge(new SequentialEdge(falseBlock.getEnd(), whileExitNode));
			firstFollower = falseBlock.getBegin();
		} else {
			block.addEdge(new FalseEdge(condition, whileExitNode));
			firstFollower = whileExitNode;
		}

		control.endControlFlowOf(block, condition, whileExitNode, condition, null);

		currentCFG.getDescriptor().addControlFlowStructure(new Loop(
				currentCFG.getNodeList(),
				condition,
				firstFollower,
				trueBlock.getBody().getNodes()));
		return new ParsedBlock(condition, block, whileExitNode);
	}

	@Override
	public ParsedBlock visitFor_stmt(
			For_stmtContext ctx) {
		// FIXME: this assumes a range-based for loop which is not always the
		// case
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of for
		NoOp exit = new NoOp(currentCFG, getLocation(filePath, ctx));
		block.addNode(exit);

		if (ctx.ASYNC() != null)
			log.warn("Async for loops are not yet supported. The for loop at line " + getLine(ctx) + " of file "
					+ filePath + " is unsoundly translated into its synchronous version.");

		Expression variable = visitStar_targets(ctx.star_targets());
		Expression collection = visitStar_expressions(ctx.star_expressions());
		SourceCodeLocation loc = getLocation(filePath, ctx);

		// the iterable is evaluated once, before the loop; each use of it (and
		// of the counter) is a distinct expression, as each node of the cfg
		// has a single parent, with a distinct location, as the results of
		// the analysis are stored for equal nodes together
		String iterable = "$iter@" + loc.getLine() + ":" + loc.getCol();
		Assignment iterable_init = new Assignment(currentCFG, loc,
				new VariableRef(currentCFG, new DerivedLocation(loc, "iter"), iterable),
				collection);
		block.addNode(iterable_init);
		String counter = "__counter_location" + loc.getLine();

		// counter = 0;
		Assignment counter_init = new Assignment(
				currentCFG,
				loc,
				new VariableRef(currentCFG, new DerivedLocation(loc, "counter-init"), counter, Int32Type.INSTANCE),
				new Int32Literal(currentCFG, loc, 0));
		block.addNode(counter_init);

		// counter < len(iterable)
		UnresolvedCall condition = new UnresolvedCall(
				currentCFG,
				loc,
				CallType.STATIC,
				null,
				"__lt__",
				new VariableRef(currentCFG, new DerivedLocation(loc, "counter-test"), counter, Int32Type.INSTANCE),
				new UnresolvedCall(
						currentCFG,
						loc,
						CallType.STATIC,
						null,
						"__len__",
						LeftToRightEvaluation.INSTANCE,
						new VariableRef(currentCFG, new DerivedLocation(loc, "iter-len"), iterable)));
		block.addNode(condition);

		// element = any element of the iterable (iterating never raises
		// IndexError), unpacked for targets like for k, v in ... (see
		// PyAssign)
		Expression element = new PyIteratedElement(currentCFG, loc,
				new VariableRef(currentCFG, new DerivedLocation(loc, "iter-element"), iterable),
				collection);
		Assignment element_assignment = variable instanceof TupleCreation
				? new PyAssign(currentCFG, loc, variable, element)
				: new Assignment(currentCFG, loc, variable, element);
		block.addNode(element_assignment);

		// counter = counter + 1;
		Assignment counter_increment = new Assignment(
				currentCFG,
				loc,
				new VariableRef(currentCFG, new DerivedLocation(loc, "counter-next"), counter, Int32Type.INSTANCE),
				new PyAddition(
						currentCFG,
						loc,
						new VariableRef(currentCFG, new DerivedLocation(loc, "counter-old"), counter,
								Int32Type.INSTANCE),
						new Int32Literal(currentCFG, loc, 1)));
		block.addNode(counter_increment);

		ParsedBlock body = visitLoopBody(ctx.block());
		block.mergeWith(body.getBody());
		block.addEdge(new SequentialEdge(iterable_init, counter_init));
		block.addEdge(new SequentialEdge(counter_init, condition));
		block.addEdge(new TrueEdge(condition, element_assignment));
		block.addEdge(new SequentialEdge(element_assignment, body.getBegin()));
		block.addEdge(new SequentialEdge(body.getEnd(), counter_increment));
		block.addEdge(new SequentialEdge(counter_increment, condition));
		block.addEdge(new FalseEdge(condition, exit));

		Collection<Statement> nodes = new HashSet<>(body.getBody().getNodes());
		nodes.add(element_assignment);
		nodes.add(counter_increment);

		control.endControlFlowOf(block, condition, exit, counter_increment, null);
		currentCFG.getDescriptor().addControlFlowStructure(new Loop(
				currentCFG.getNodeList(),
				condition,
				exit,
				nodes));
		return new ParsedBlock(iterable_init, block, exit);
	}

	@Override
	public ParsedBlock visitTry_stmt(
			Try_stmtContext ctx) {
		log.warn("Exceptions are not yet supported. The try block at line " + getLine(ctx) + " of file " + filePath
				+ " is unsoundly translated considering only the code in the try block");
		return visitBlock(ctx.block());
	}

	@Override
	public ParsedBlock visitWith_stmt(
			With_stmtContext ctx) {
		if (ctx.ASYNC() != null)
			log.warn("Async with statements are not yet supported. The with statement at line " + getLine(ctx)
					+ " of file " + filePath + " is unsoundly translated into its synchronous version.");

		int withSize = ctx.with_item().size();
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		ParsedBlock curr = visitWith_item(ctx.with_item(0));
		Statement start = curr.getBegin();
		Statement prev = curr.getEnd();
		block.mergeWith(curr.getBody());

		for (int i = 1; i < withSize; i++) {
			curr = visitWith_item(ctx.with_item(i));
			block.mergeWith(curr.getBody());
			block.addEdge(new SequentialEdge(prev, curr.getBegin()));
			prev = curr.getEnd();
		}

		ParsedBlock suite = visitBlock(ctx.block());
		block.mergeWith(suite.getBody());
		block.addEdge(new SequentialEdge(prev, suite.getBegin()));

		return new ParsedBlock(start, block, suite.getEnd());
	}

	@Override
	public ParsedBlock visitWith_item(
			With_itemContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement test = visitExpression(ctx.expression());
		block.addNode(test);
		Statement expr = test;
		if (ctx.star_target() != null) {
			expr = visitTarget(ctx.star_target());
			block.addNode(expr);
			block.addEdge(new SequentialEdge(test, expr));
		}
		return new ParsedBlock(test, block, expr);
	}

	@Override
	public Expression visitExpression(
			ExpressionContext ctx) {
		if (ctx.lambdef() != null)
			return visitLambdef(ctx.lambdef());
		else if (ctx.IF() != null) {
			Expression trueCase = visitDisjunction(ctx.disjunction(0));
			Expression booleanGuard = visitDisjunction(ctx.disjunction(1));
			Expression falseCase = visitExpression(ctx.expression());

			return new PyTernaryOperator(currentCFG, getLocation(filePath, ctx), booleanGuard, trueCase, falseCase);
		} else
			return visitDisjunction(ctx.disjunction(0));
	}

	/**
	 * Translates {@code lambda params: body} into a function of the current
	 * unit, named {@code $lambda@line:col}, whose body is {@code return body};
	 * the value of the lambda is the function, as for a {@code def} (see
	 * {@link PyFunctionLiteral}). The variables of the enclosing function that
	 * the body uses are not known inside it (closures are not modeled). When
	 * pandas is used, a lambda without parameters is instead a symbolic value,
	 * as expected by the dataframe analyses.
	 */
	@Override
	public Expression visitLambdef(
			LambdefContext ctx) {
		SourceCodeLocation loc = getLocation(filePath, ctx);
		if (ctx.lambda_params() == null
				&& LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			return new LambdaExpression(new ArrayList<Expression>(), visitExpression(ctx.expression()), currentCFG,
					loc);

		String name = "$lambda@" + loc.getLine() + ":" + loc.getCol();
		List<PyParameter> params = new ArrayList<>();
		List<Pair<PyParameter, Default_assignmentContext>> defaults = new ArrayList<>();
		if (ctx.lambda_params() != null)
			lambdaParameters(ctx.lambda_params().lambda_parameters(), params, defaults);
		CodeMemberDescriptor descriptor = new CodeMemberDescriptor(loc, currentUnit, false, name,
				params.toArray(PyParameter[]::new));
		NodeList<CFG, Statement, Edge> list = new NodeList<>(new SequentialEdge());
		Collection<Statement> entrypoints = new HashSet<>();
		PyCFG lambda = new PyCFG(descriptor, entrypoints, list);
		PyStatementParser parser = new PyStatementParser(program, filePath, currentUnit, lambda,
				new ControlFlowTracker(), localImports);
		// the default values are parsed in the cfg of the lambda, as for
		// functions (see PyCodeMemberParser)
		for (Pair<PyParameter, Default_assignmentContext> def : defaults)
			def.getLeft().setDefaultValue(parser.visitExpression(def.getRight().expression()));

		Return ret = new Return(lambda, loc, parser.visitExpression(ctx.expression()));
		list.addNode(ret);
		entrypoints.add(ret);
		currentUnit.addCodeMember(lambda);
		return new PyFunctionLiteral(currentCFG, loc, lambda);
	}

	// the parameters of a lambda: positional ones (possibly with a default
	// value, collected in defaults), *args, keyword-only ones and **kwargs
	private void lambdaParameters(
			Lambda_parametersContext ctx,
			List<PyParameter> params,
			List<Pair<PyParameter, Default_assignmentContext>> defaults) {
		List<Pair<Lambda_paramContext, Default_assignmentContext>> positional = new ArrayList<>();
		if (ctx.lambda_slash_no_default() != null)
			for (Lambda_param_no_defaultContext p : ctx.lambda_slash_no_default().lambda_param_no_default())
				positional.add(Pair.of(p.lambda_param(), null));
		else if (ctx.lambda_slash_with_default() != null) {
			for (Lambda_param_no_defaultContext p : ctx.lambda_slash_with_default().lambda_param_no_default())
				positional.add(Pair.of(p.lambda_param(), null));
			for (Lambda_param_with_defaultContext p : ctx.lambda_slash_with_default().lambda_param_with_default())
				positional.add(Pair.of(p.lambda_param(), p.default_assignment()));
		}
		for (Lambda_param_no_defaultContext p : ctx.lambda_param_no_default())
			positional.add(Pair.of(p.lambda_param(), null));
		for (Lambda_param_with_defaultContext p : ctx.lambda_param_with_default())
			positional.add(Pair.of(p.lambda_param(), p.default_assignment()));
		for (Pair<Lambda_paramContext, Default_assignmentContext> p : positional) {
			PyParameter parameter = new PyParameter(getLocation(filePath, p.getLeft()),
					p.getLeft().name().getText());
			params.add(parameter);
			if (p.getRight() != null)
				defaults.add(Pair.of(parameter, p.getRight()));
		}

		Lambda_star_etcContext star = ctx.lambda_star_etc();
		if (star == null)
			return;
		if (star.lambda_param_no_default() != null)
			params.add(new VarPositionalParameter(getLocation(filePath, star.lambda_param_no_default()),
					star.lambda_param_no_default().lambda_param().name().getText()));
		for (Lambda_param_maybe_defaultContext p : star.lambda_param_maybe_default()) {
			PyParameter parameter = new KeywordOnlyParameter(getLocation(filePath, p.lambda_param()),
					p.lambda_param().name().getText());
			params.add(parameter);
			if (p.default_assignment() != null)
				defaults.add(Pair.of(parameter, p.default_assignment()));
		}
		if (star.lambda_kwds() != null)
			params.add(new VarKeywordParameter(getLocation(filePath, star.lambda_kwds()),
					star.lambda_kwds().lambda_param_no_default().lambda_param().name().getText()));
	}

	@Override
	public Expression visitDisjunction(
			DisjunctionContext ctx) {
		int nConjunction = ctx.conjunction().size();
		if (nConjunction == 1) {
			return visitConjunction(ctx.conjunction(0));
		} else if (nConjunction == 2) {
			return new PyOr(currentCFG, getLocation(filePath, ctx),
					visitConjunction(ctx.conjunction(0)),
					visitConjunction(ctx.conjunction(1)));
		} else {
			Expression temp = new PyOr(currentCFG, getLocation(filePath, ctx),
					visitConjunction(ctx.conjunction(nConjunction - 2)),
					visitConjunction(ctx.conjunction(nConjunction - 1)));
			nConjunction = nConjunction - 2;
			while (nConjunction > 0) {
				temp = new PyOr(currentCFG, getLocation(filePath, ctx),
						visitConjunction(ctx.conjunction(--nConjunction)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitConjunction(
			ConjunctionContext ctx) {
		int nInversion = ctx.inversion().size();
		if (nInversion == 1) {
			return visitInversion(ctx.inversion(0));
		} else if (nInversion == 2) {
			return new PyAnd(currentCFG, getLocation(filePath, ctx),
					visitInversion(ctx.inversion(0)),
					visitInversion(ctx.inversion(1)));
		} else {
			Expression temp = new PyAnd(currentCFG, getLocation(filePath, ctx),
					visitInversion(ctx.inversion(nInversion - 2)),
					visitInversion(ctx.inversion(nInversion - 1)));
			nInversion = nInversion - 2;
			while (nInversion > 0) {
				temp = new PyAnd(currentCFG, getLocation(filePath, ctx),
						visitInversion(ctx.inversion(--nInversion)),
						temp);
			}

			return temp;
		}
	}

	@Override
	public Expression visitInversion(
			InversionContext ctx) {
		if (ctx.NOT() != null)
			return new PyNot(currentCFG, getLocation(filePath, ctx), visitInversion(ctx.inversion()));
		else
			return visitComparison(ctx.comparison());
	}

	@Override
	public Expression visitComparison(
			ComparisonContext ctx) {
		Expression result = visitBitwise_or(ctx.bitwise_or());
		for (Compare_op_bitwise_or_pairContext pair : ctx.compare_op_bitwise_or_pair())
			result = visitCompare_op_bitwise_or_pair(pair, result);
		return result;
	}

	private Expression visitCompare_op_bitwise_or_pair(
			Compare_op_bitwise_or_pairContext ctx,
			Expression left) {
		if (ctx.eq_bitwise_or() != null)
			return new PyEquals(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.eq_bitwise_or().bitwise_or()));
		else if (ctx.noteq_bitwise_or() != null)
			return new PyNotEqual(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.noteq_bitwise_or().bitwise_or()));
		else if (ctx.lte_bitwise_or() != null)
			return new PyLessOrEqual(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.lte_bitwise_or().bitwise_or()));
		else if (ctx.lt_bitwise_or() != null)
			return new PyLessThan(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.lt_bitwise_or().bitwise_or()));
		else if (ctx.gte_bitwise_or() != null)
			return new PyGreaterOrEqual(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.gte_bitwise_or().bitwise_or()));
		else if (ctx.gt_bitwise_or() != null)
			return new PyGreaterThan(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.gt_bitwise_or().bitwise_or()));
		else if (ctx.notin_bitwise_or() != null)
			return new PyNot(currentCFG, getLocation(filePath, ctx),
					new PyIn(currentCFG, getLocation(filePath, ctx), left,
							visitBitwise_or(ctx.notin_bitwise_or().bitwise_or())));
		else if (ctx.in_bitwise_or() != null)
			return new PyIn(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.in_bitwise_or().bitwise_or()));
		else if (ctx.isnot_bitwise_or() != null)
			return new PyNot(currentCFG, getLocation(filePath, ctx),
					new PyIs(currentCFG, getLocation(filePath, ctx), left,
							visitBitwise_or(ctx.isnot_bitwise_or().bitwise_or())));
		else if (ctx.is_bitwise_or() != null)
			return new PyIs(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.is_bitwise_or().bitwise_or()));
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitBitwise_or(
			Bitwise_orContext ctx) {
		if (ctx.bitwise_or() == null)
			return visitBitwise_xor(ctx.bitwise_xor());
		else
			return new PyBitwiseOr(currentCFG, getLocation(filePath, ctx),
					visitBitwise_or(ctx.bitwise_or()),
					visitBitwise_xor(ctx.bitwise_xor()));
	}

	@Override
	public Expression visitBitwise_xor(
			Bitwise_xorContext ctx) {
		if (ctx.bitwise_xor() == null)
			return visitBitwise_and(ctx.bitwise_and());
		else
			return new PyBitwiseXor(currentCFG, getLocation(filePath, ctx),
					visitBitwise_xor(ctx.bitwise_xor()),
					visitBitwise_and(ctx.bitwise_and()));
	}

	@Override
	public Expression visitBitwise_and(
			Bitwise_andContext ctx) {
		if (ctx.bitwise_and() == null)
			return visitShift_expr(ctx.shift_expr());
		else
			return new PyBitwiseAnd(currentCFG, getLocation(filePath, ctx),
					visitBitwise_and(ctx.bitwise_and()),
					visitShift_expr(ctx.shift_expr()));
	}

	public Expression visitLeft_shift(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else
			return new PyBitwiseLeftShift(currentCFG, getLocation(filePath, ctx),
					visitShift_expr(ctx.shift_expr()),
					visitSum(ctx.sum()));
	}

	public Expression visitRight_shift(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else
			return new PyBitwiseRIghtShift(currentCFG, getLocation(filePath, ctx),
					visitShift_expr(ctx.shift_expr()),
					visitSum(ctx.sum()));
	}

	@Override
	public Expression visitShift_expr(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else if (ctx.LEFTSHIFT() != null)
			return visitLeft_shift(ctx);
		else if (ctx.RIGHTSHIFT() != null)
			return visitRight_shift(ctx);
		throw new UnsupportedStatementException();
	}

	public Expression visitMinus(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else
			return new PySubtraction(currentCFG, getLocation(filePath, ctx),
					visitSum(ctx.sum()),
					visitTerm(ctx.term()));
	}

	public Expression visitAdd(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else
			return new PyAddition(currentCFG, getLocation(filePath, ctx),
					visitSum(ctx.sum()),
					visitTerm(ctx.term()));
	}

	@Override
	public Expression visitSum(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else if (ctx.MINUS() != null)
			return visitMinus(ctx);
		else if (ctx.PLUS() != null)
			return visitAdd(ctx);
		throw new UnsupportedStatementException();
	}

	public Expression visitMul(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMultiplication(currentCFG, getLocation(filePath, ctx),
					visitTerm(ctx.term()),
					visitFactor(ctx.factor()));
	}

	public Expression visitMat_mul(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMatMul(currentCFG, getLocation(filePath, ctx),
					visitTerm(ctx.term()),
					visitFactor(ctx.factor()));
	}

	public Expression visitDiv(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyDivision(currentCFG, getLocation(filePath, ctx),
					visitTerm(ctx.term()),
					visitFactor(ctx.factor()));
	}

	public Expression visitMod(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyRemainder(currentCFG, getLocation(filePath, ctx),
					visitTerm(ctx.term()),
					visitFactor(ctx.factor()));
	}

	public Expression visitFloorDiv(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyFloorDiv(currentCFG, getLocation(filePath, ctx),
					visitTerm(ctx.term()),
					visitFactor(ctx.factor()));
	}

	@Override
	public Expression visitTerm(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else if (ctx.STAR() != null)
			return visitMul(ctx);
		else if (ctx.AT() != null)
			return visitMat_mul(ctx);
		else if (ctx.SLASH() != null)
			return visitDiv(ctx);
		else if (ctx.PERCENT() != null)
			return visitMod(ctx);
		else if (ctx.DOUBLESLASH() != null)
			return visitFloorDiv(ctx);
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitFactor(
			FactorContext ctx) {
		if (ctx.power() != null)
			return visitPower(ctx.power());
		else if (ctx.TILDE() != null)
			return new PyBitwiseNot(currentCFG, getLocation(filePath, ctx),
					visitFactor(ctx.factor()));
		else if (ctx.MINUS() != null)
			return new PyNegation(currentCFG, getLocation(filePath, ctx),
					visitFactor(ctx.factor()));
		return visitFactor(ctx.factor());
	}

	@Override
	public Expression visitPower(
			PowerContext ctx) {
		if (ctx.DOUBLESTAR() != null)
			return new PyPower(currentCFG, getLocation(filePath, ctx),
					visitAtom_expr(ctx.await_primary()),
					visitFactor(ctx.factor()));
		else
			return visitAtom_expr(ctx.await_primary());
	}

	public Expression visitAtom_expr(
			Await_primaryContext ctx) {
		if (ctx.AWAIT() != null)
			throw new UnsupportedStatementException("await is not supported");
		return visitPrimary(ctx.primary());
	}

	@Override
	public Expression visitPrimary(
			PrimaryContext ctx) {
		List<PrimaryContext> chain = new ArrayList<>();
		PrimaryContext base = ctx;
		while (base.primary() != null) {
			chain.add(0, base);
			base = base.primary();
		}

		Expression access = visitAtom(base.atom());
		String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
		Expression previous_access = null;
		// whether the last frame was an attribute access (x.name)
		boolean attribute = false;

		for (PrimaryContext frame : chain) {
			if (frame.DOT() != null) {
				last_name = frame.name().getText();
				previous_access = access;
				attribute = true;
				Expression member = moduleMember(access, last_name, getLocation(filePath, frame));
				access = member != null ? member : attribute(access, last_name, getLocation(filePath, frame));
			} else if (frame.LPAR() != null || frame.genexp() != null) {
				List<Expression> arguments;
				if (frame.genexp() == null)
					arguments = extractArguments(frame.arguments());
				else if (consumesGenerator(last_name, access instanceof PyAccessInstanceGlobal
						|| (attribute && isMethodReceiver(previous_access))))
					// e.g. sum(x for x in xs)
					arguments = new ArrayList<>(List.of(visitGenexpArgument(frame.genexp())));
				else
					throw new UnsupportedStatementException("generator expression calls are not supported");

				if (last_name == null) {
					// the callee is not a name, e.g. d[key]() or f()()
					List<Expression> args = convertAssignmentsToByNameParameters(arguments);
					access = new PyValueCall(currentCFG, getLocation(filePath, frame), access,
							args.toArray(Expression[]::new));
					previous_access = null;
					attribute = false;
					continue;
				}

				List<Expression> pars = arguments;
				String method_name = last_name;
				// x.name(...) is a method call on x, and x is passed as the
				// receiver, unless x is a namespace (x.name is then just a
				// function or a class)
				boolean instance = access instanceof PyAccessInstanceGlobal
						|| (attribute && isMethodReceiver(previous_access));
				if (instance)
					pars.add(0, previous_access);

				pars = convertAssignmentsToByNameParameters(pars);
				Unit cu = program.getUnit(method_name);
				if (cu == null) {
					cu = program.getUnit(access.toString().replace("::", "."));
					if (cu == null) {
						String unitName = imports.get(access.toString());
						if (unitName != null)
							cu = program.getUnit(unitName);
					}
					if (cu != null) {
						for (Expression par : pars)
							if (par instanceof AccessInstanceGlobal)
								pars.remove(par);
					}
				}
				if (cu != null && cu instanceof ClassUnit) {
					if (instance)
						// x.Cls(...) builds a Cls, with no receiver
						pars.remove(0);
					access = new PyNewObj(
							currentCFG,
							getLocation(filePath, frame),
							"__init__",
							PyClassType.register(cu.getName(), (ClassUnit) cu),
							pars.toArray(Expression[]::new));
				} else if (attribute && localImports.module(dotted(previous_access)) != null) {
					// m.f(...): the function f of the local module m
					access = new PyModuleCall(currentCFG, getLocation(filePath, frame),
							localImports.module(dotted(previous_access)), method_name, pars.toArray(Expression[]::new));
				} else if (!attribute && previous_access == null && localImports.member(method_name) != null) {
					// f(...), with f imported from a local module
					LocalImports.Member member = localImports.member(method_name);
					access = new PyModuleCall(currentCFG, getLocation(filePath, frame), member.unit(), member.name(),
							pars.toArray(Expression[]::new));
				} else if (attribute && classNames(previous_access) != null) {
					// C.m(...): the function m of the class, with no receiver
					access = new PyClassFunctionCall(
							currentCFG,
							getLocation(filePath, frame),
							classNames(previous_access),
							method_name,
							LeftToRightEvaluation.INSTANCE,
							pars.toArray(Expression[]::new));
				} else if (!instance && !attribute && method_name.equals("abs") && pars.size() == 1
						&& !(pars.get(0) instanceof NamedParameterExpression) && isBuiltin("abs")) {
					access = new PyAbs(currentCFG, getLocation(filePath, frame), pars.get(0));
				} else if (!instance && !attribute && (method_name.equals("min") || method_name.equals("max"))
						&& minMaxArguments(pars) && isBuiltin(method_name)) {
					// min(a, b, ...), min(iterable), min(iterable, default=d)
					Expression dflt = pars.stream().filter(NamedParameterExpression.class::isInstance).findFirst()
							.map(p -> ((NamedParameterExpression) p).getSubExpression()).orElse(null);
					access = new PyMinMax(currentCFG, getLocation(filePath, frame), method_name.equals("max"),
							pars.stream().filter(p -> !(p instanceof NamedParameterExpression))
									.toArray(Expression[]::new),
							dflt);
				} else if (!instance && method_name.equals("len") && pars.size() == 1) {
					access = new PyLength(currentCFG, getLocation(filePath, frame), pars.get(0));
				} else if (!instance && method_name.equals("divmod") && pars.size() == 2
						&& pars.stream().noneMatch(NamedParameterExpression.class::isInstance)) {
					access = new PyDivMod(currentCFG, getLocation(filePath, frame), pars.get(0), pars.get(1));
				} else {
					access = instance
							? new PyMethodCall(
									currentCFG,
									getLocation(filePath, frame),
									method_name,
									LeftToRightEvaluation.INSTANCE,
									pars.toArray(Expression[]::new))
							: !attribute && previous_access == null
									// f(...): f might be a variable holding
									// functions
									? new PyNameCall(
											currentCFG,
											getLocation(filePath, frame),
											method_name,
											LeftToRightEvaluation.INSTANCE,
											pars.toArray(Expression[]::new))
									: new UnresolvedCall(
											currentCFG,
											getLocation(filePath, frame),
											CallType.STATIC,
											// e.g. bytes.fromhex(s) is looked
											// up in Bytes
											attribute && previous_access instanceof VariableRef
													? BUILTIN_CLASSES
															.get(((VariableRef) previous_access).getName())
													: null,
											method_name,
											LeftToRightEvaluation.INSTANCE,
											pars.toArray(Expression[]::new));
					if (method_name.equals("super") && pars.isEmpty()) {
						// if super() is inside an instance method
						if (this.currentCFG.getDescriptor().isInstance()) {
							VariableTableEntry vte = currentCFG.getDescriptor().getVariables().get(0);

							Expression[] expressions = new Expression[2];
							expressions[0] = new PyTypeLiteral(this.currentCFG, getLocation(filePath, frame),
									this.currentUnit);
							expressions[1] = new VariableRef(this.currentCFG, getLocation(filePath, frame),
									vte.getName());
							access = new SimpleSuperUnresolvedCall(
									currentCFG,
									getLocation(filePath, frame),
									instance ? CallType.UNKNOWN : CallType.STATIC,
									null,
									method_name,
									expressions);
						}
					}
				}
				last_name = null;
				previous_access = null;
				attribute = false;
			} else if (frame.LSQB() != null) {
				previous_access = access;
				last_name = null;
				attribute = false;
				List<Expression> indexes = extractExpressionsFromSlices(frame.slices());
				if (indexes.size() == 1)
					access = new PySingleArrayAccess(
							currentCFG,
							getLocation(filePath, frame),
							Untyped.INSTANCE,
							access,
							indexes.get(0));
				else if (indexes.size() == 2)
					access = new PyDoubleArrayAccess(
							currentCFG,
							getLocation(filePath, frame),
							Untyped.INSTANCE,
							access,
							indexes.get(0),
							indexes.get(1));
				else
					throw new UnsupportedStatementException("subscripts with more than two indexes are not supported");
			} else
				throw new UnsupportedStatementException();
		}
		return access;
	}

	/**
	 * Whether {@code receiver.name(...)} is a method call on {@code receiver},
	 * that has to be passed as first argument: this is the case unless
	 * {@code receiver} is a namespace (see {@link #isNamespace(Expression)}) or
	 * a call to {@code super()}, that is handled separately.
	 */
	private boolean isMethodReceiver(
			Expression receiver) {
		if (receiver instanceof SimpleSuperUnresolvedCall
				|| (receiver instanceof UnresolvedCall && ((UnresolvedCall) receiver).getTargetName().equals("super")))
			return false;
		return !isNamespace(receiver);
	}

	/**
	 * Builds the access to the attribute {@code name} of {@code receiver}: a
	 * {@link PyAttributeAccess} for values, or a call to
	 * {@code __getattribute__} for namespaces (e.g. {@code math.floor}, whose
	 * calls are resolved by name, see {@link #isNamespace(Expression)}).
	 */
	private Expression attribute(
			Expression receiver,
			String name,
			CodeLocation location) {
		if (isNamespace(receiver))
			return new UnresolvedCall(
					currentCFG,
					location,
					CallType.INSTANCE,
					null,
					"__getattribute__",
					receiver,
					new PyStringLiteral(currentCFG, location, name, "'"));
		return new PyAttributeAccess(currentCFG, location, receiver, name);
	}

	/**
	 * Whether {@code expr} denotes a namespace rather than a value: a name
	 * bound by an import, the name of a class or of a library, or an attribute
	 * of one of those (e.g. {@code os.path}).
	 */
	// the names of the class of the program that the expression might refer
	// to, either defined in this module (see PyClassRef) or imported (from m
	// import C), if any: imported classes are looked up during the analysis
	// (see PyClassFunctionCall)
	private List<String> classNames(
			Expression expr) {
		if (expr instanceof PyClassRef)
			return List.of(((PyClassRef) expr).getClassUnit().getName());
		if (!(expr instanceof VariableRef) || !imports.containsKey(((VariableRef) expr).getName()))
			return null;
		String name = ((VariableRef) expr).getName();
		return List.of(imports.get(name), name);
	}

	/**
	 * Yields the dotted name of an expression made of names and of attributes
	 * of namespaces (e.g. {@code a.b}), or {@code null}.
	 */
	private static String dotted(
			Expression expr) {
		if (expr instanceof VariableRef)
			return ((VariableRef) expr).getName();
		if (expr instanceof UnresolvedCall && ((UnresolvedCall) expr).getTargetName().equals("__getattribute__")) {
			Expression[] params = ((UnresolvedCall) expr).getParameters();
			String receiver = params.length == 2 && params[1] instanceof PyStringLiteral ? dotted(params[0]) : null;
			return receiver == null ? null : receiver + "." + ((PyStringLiteral) params[1]).getValue();
		}
		return null;
	}

	/**
	 * Yields the value of {@code receiver.name} if the receiver is a local
	 * module and {@code name} is not one of its submodules: a class of the
	 * module, or a function of the module (see {@link PyFunctionRef}).
	 * Otherwise, it yields {@code null}.
	 */
	private Expression moduleMember(
			Expression receiver,
			String name,
			CodeLocation location) {
		String path = dotted(receiver);
		String module = localImports.module(path);
		if (module == null || localImports.module(path + "." + name) != null)
			return null;
		Unit unit = program.getUnit(name);
		if (unit instanceof ClassUnit && ((ClassUnit) unit).getLocation() instanceof SourceCodeLocation
				&& FilenameUtils
						.removeExtension(((SourceCodeLocation) ((ClassUnit) unit).getLocation()).getSourceFile())
						.equals(module))
			// a class of the module
			return new PyClassRef(currentCFG, location, (ClassUnit) unit);
		return new PyFunctionRef(currentCFG, location, path + "." + name, module, name);
	}

	private boolean isNamespace(
			Expression expr) {
		if (expr instanceof VariableRef) {
			String name = ((VariableRef) expr).getName();
			return namespaces.contains(name) || program.getUnit(name) != null || BUILTIN_CLASSES.containsKey(name)
					|| localImports.module(name) != null;
		}
		if (expr instanceof UnresolvedCall) {
			UnresolvedCall call = (UnresolvedCall) expr;
			if (call.getTargetName().equals("__getattribute__") && call.getParameters().length == 2)
				return isNamespace(call.getParameters()[0]);
		}
		return false;
	}

	private List<Expression> convertAssignmentsToByNameParameters(
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

	public List<Expression> extractArguments(
			ArgumentsContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.args() == null)
			return result;
		for (int i = 0; i < ctx.args().getChildCount(); i++) {
			ParseTree child = ctx.args().getChild(i);
			if (child instanceof ExpressionContext)
				result.add(visitExpression((ExpressionContext) child));
			else if (child instanceof Assignment_expressionContext)
				result.add(visitAssignment_expression((Assignment_expressionContext) child));
			else if (child instanceof Starred_expressionContext)
				result.add(starArgument((Starred_expressionContext) child));
			else if (child instanceof KwargsContext)
				extractKwargs((KwargsContext) child, result);
		}
		return result;
	}

	private void extractKwargs(
			KwargsContext ctx,
			List<Expression> result) {
		for (Kwarg_or_starredContext k : ctx.kwarg_or_starred())
			result.add(visitKwarg_or_starred(k));
		if (!ctx.kwarg_or_double_starred().isEmpty())
			throw new UnsupportedStatementException("** kwargs unpacking is not supported");
	}

	@Override
	public Expression visitKwarg_or_starred(
			Kwarg_or_starredContext ctx) {
		if (ctx.name() != null)
			return new PyAssign(currentCFG, getLocation(filePath, ctx),
					new VariableRef(currentCFG, getLocation(filePath, ctx.name()), ctx.name().getText()),
					visitExpression(ctx.expression()));
		return starArgument(ctx.starred_expression());
	}

	// *xs as an argument of a call
	private Expression starArgument(
			Starred_expressionContext ctx) {
		return new StarArgument(currentCFG, getLocation(filePath, ctx), visitExpression(ctx.expression()));
	}

	@Override
	public Expression visitStarred_expression(
			Starred_expressionContext ctx) {
		return new StarExpression(currentCFG, getLocation(filePath, ctx), visitExpression(ctx.expression()));
	}

	@Override
	public Expression visitAssignment_expression(
			Assignment_expressionContext ctx) {
		return new PyAssign(currentCFG, getLocation(filePath, ctx),
				new VariableRef(currentCFG, getLocation(filePath, ctx.name()), ctx.name().getText()),
				visitExpression(ctx.expression()));
	}

	@Override
	public Expression visitNamed_expression(
			Named_expressionContext ctx) {
		if (ctx.assignment_expression() != null)
			return visitAssignment_expression(ctx.assignment_expression());
		return visitExpression(ctx.expression());
	}

	private List<Expression> extractExpressionsFromSlices(
			SlicesContext ctx) {
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < ctx.getChildCount(); i++) {
			ParseTree child = ctx.getChild(i);
			if (child instanceof SliceContext)
				result.add(visitSlice((SliceContext) child));
			else if (child instanceof Starred_expressionContext)
				result.add(visitStarred_expression((Starred_expressionContext) child));
		}
		return result;
	}

	@Override
	public Expression visitSlice(
			SliceContext ctx) {
		if (ctx.named_expression() != null)
			return visitNamed_expression(ctx.named_expression());

		SourceCodeLocation loc = getLocation(filePath, ctx);
		// the missing parts have distinct locations, as each node of the cfg
		// must be distinct from the others
		Expression start = new Empty(currentCFG, new DerivedLocation(loc, "start"));
		Expression stop = new Empty(currentCFG, new DerivedLocation(loc, "stop"));
		Expression step = new Empty(currentCFG, new DerivedLocation(loc, "step"));
		int colonsSeen = 0;
		for (int i = 0; i < ctx.getChildCount(); i++) {
			ParseTree child = ctx.getChild(i);
			if (child instanceof TerminalNode)
				colonsSeen++;
			else if (child instanceof ExpressionContext) {
				Expression e = visitExpression((ExpressionContext) child);
				if (colonsSeen == 0)
					start = e;
				else if (colonsSeen == 1)
					stop = e;
				else
					step = e;
			}
		}
		return new RangeValue(currentCFG, loc, start, stop, step);
	}

	private List<Expression> extractExpressionsFromStar_named_expressions(
			Star_named_expressionsContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null)
			return result;
		for (Star_named_expressionContext e : ctx.star_named_expression())
			result.add(visitStar_named_expression(e));
		return result;
	}

	@Override
	public Expression visitStar_named_expression(
			Star_named_expressionContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(filePath, ctx), visitBitwise_or(ctx.bitwise_or()));
		return visitNamed_expression(ctx.named_expression());
	}

	private List<Pair<Expression, Expression>> extractPairsFromDict(
			Double_starred_kvpairsContext ctx) {
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx == null)
			return result;
		for (Double_starred_kvpairContext e : ctx.double_starred_kvpair()) {
			if (e.kvpair() == null)
				// **m: no key (see DictionaryCreation)
				result.add(Pair.of(null, visitBitwise_or(e.bitwise_or())));
			else
				result.add(Pair.of(visitExpression(e.kvpair().expression(0)),
						visitExpression(e.kvpair().expression(1))));
		}
		return result;
	}

	/**
	 * Translates {@code [element for x in iterable if c ...]}: the variables of
	 * the comprehension are local to it, so they are renamed (e.g.
	 * {@code x$comp@3:4}) in its clauses and element. The first iterable is
	 * evaluated outside of the comprehension.
	 */
	private Expression visitListComprehension(
			ListcompContext ctx) {
		return comprehension(ctx, ctx.for_if_clauses(), () -> visitNamed_expression(ctx.named_expression()));
	}

	/**
	 * Translates the generator expression {@code (element for x in iterable if
	 * c ...)} passed to a function that consumes it right away (see
	 * {@link #consumesGenerator(String, boolean)}): as the generator is fully
	 * iterated during the call, it is translated as the list comprehension
	 * {@code [element for x in iterable if c ...]}. The only difference is that
	 * functions like {@code any} might stop the iteration early: the elements
	 * after that point are not computed by python, so the errors raised by the
	 * comprehension are an over-approximation.
	 */
	private Expression visitGenexpArgument(
			GenexpContext ctx) {
		return comprehension(ctx, ctx.for_if_clauses(),
				() -> ctx.assignment_expression() != null ? visitAssignment_expression(ctx.assignment_expression())
						: visitExpression(ctx.expression()));
	}

	// the builtins that consume a generator passed to them by iterating it,
	// and that behave in the same way when given a list instead
	private static final Set<String> GENERATOR_CONSUMERS = Set.of("sum", "all", "any", "min", "max", "list", "tuple",
			"set", "frozenset", "sorted", "dict");

	/**
	 * Whether a generator expression passed to {@code name} can be translated
	 * as a list comprehension (see
	 * {@link #visitGenexpArgument(GenexpContext)}): this holds for the builtins
	 * consuming iterables (e.g. {@code sum}), and for {@code str.join}, unless
	 * the program defines a function with the same name. Other functions might
	 * use the generator as such (e.g. {@code next}, or {@code len}, that raises
	 * {@code TypeError} on generators).
	 */
	// whether name is the builtin function, that is, the program does not
	// define a function with that name
	private boolean isBuiltin(
			String name) {
		return program.getCodeMembersByName(name).stream().allMatch(NativeCFG.class::isInstance);
	}

	// whether min/max are called with arguments that are modeled: positional
	// ones and default= (key= is not modeled), with no starred argument
	private static boolean minMaxArguments(
			List<Expression> pars) {
		for (Expression p : pars)
			if (p instanceof StarExpression
					|| (p instanceof NamedParameterExpression
							&& !((NamedParameterExpression) p).getParameterName().equals("default")))
				return false;
		return true;
	}

	private boolean consumesGenerator(
			String name,
			boolean method) {
		if (name == null)
			return false;
		if (method)
			return name.equals("join");
		return GENERATOR_CONSUMERS.contains(name) && program.getCodeMembersByName(name).stream()
				.allMatch(NativeCFG.class::isInstance);
	}

	private Expression comprehension(
			ParserRuleContext ctx,
			For_if_clausesContext forIfClauses,
			Supplier<Expression> elementVisitor) {
		SourceCodeLocation loc = getLocation(filePath, ctx);
		Map<String, String> scope = new HashMap<>();
		List<PyListComprehension.Clause> clauses = new ArrayList<>();
		comprehensions.push(scope);
		try {
			for (For_if_clauseContext clause : forIfClauses.for_if_clause()) {
				if (clause.ASYNC() != null)
					throw new UnsupportedStatementException("async comprehensions are not supported");
				// the iterable sees the variables of the previous clauses
				Expression iterable = visitDisjunction(clause.disjunction(0));
				Expression target = comprehensionTarget(clause.star_targets(), scope, loc);
				List<Expression> conditions = new ArrayList<>();
				for (int i = 1; i < clause.disjunction().size(); i++)
					conditions.add(visitDisjunction(clause.disjunction(i)));
				clauses.add(new PyListComprehension.Clause(target, iterable, conditions));
			}
			Expression element = elementVisitor.get();
			return new PyListComprehension(currentCFG, loc, element, clauses);
		} finally {
			comprehensions.pop();
		}
	}

	/**
	 * The target of a clause of a comprehension, whose names are variables
	 * local to the comprehension (added to {@code scope}): a name, or a tuple
	 * or list of targets, possibly starred (e.g. {@code k, v} in
	 * {@code for k, v in pairs}, see {@link PyListComprehension.Clause}).
	 */
	private Expression comprehensionTarget(
			Star_targetsContext ctx,
			Map<String, String> scope,
			SourceCodeLocation loc) {
		if (ctx.star_target().size() == 1)
			return comprehensionTarget(ctx.star_target(0), scope, loc);
		List<Expression> elements = new ArrayList<>();
		for (Star_targetContext t : ctx.star_target())
			elements.add(comprehensionTarget(t, scope, loc));
		return new TupleCreation(currentCFG, getLocation(filePath, ctx), elements.toArray(Expression[]::new));
	}

	private Expression comprehensionTarget(
			Star_targetContext ctx,
			Map<String, String> scope,
			SourceCodeLocation loc) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(filePath, ctx),
					comprehensionTarget(ctx.star_target(), scope, loc));
		return comprehensionTarget(ctx.target_with_star_atom(), scope, loc);
	}

	private Expression comprehensionTarget(
			Target_with_star_atomContext ctx,
			Map<String, String> scope,
			SourceCodeLocation loc) {
		Star_atomContext atom = ctx.star_atom();
		if (atom == null)
			// e.g. o.x or l[i]
			throw new UnsupportedStatementException(
					"attributes and subscripts as targets of comprehensions are not supported");
		if (atom.name() != null) {
			String name = atom.name().getText();
			String local = name + "$comp@" + loc.getLine() + ":" + loc.getCol();
			scope.put(name, local);
			return new VariableRef(currentCFG, getLocation(filePath, atom), local);
		}
		if (atom.target_with_star_atom() != null)
			// (x)
			return comprehensionTarget(atom.target_with_star_atom(), scope, loc);
		// a nested target, e.g. (b, c) in for a, (b, c) in ...
		List<Star_targetContext> targets = atom.star_targets_tuple_seq() != null
				? atom.star_targets_tuple_seq().star_target()
				: atom.star_targets_list_seq() != null ? atom.star_targets_list_seq().star_target() : List.of();
		if (targets.isEmpty())
			throw new UnsupportedStatementException("Empty unpacking targets are not supported");
		List<Expression> elements = new ArrayList<>();
		for (Star_targetContext t : targets)
			elements.add(comprehensionTarget(t, scope, loc));
		return new TupleCreation(currentCFG, getLocation(filePath, atom), elements.toArray(Expression[]::new));
	}

	// the reference to name, at the location of ctx: a variable of a
	// comprehension, a builtin function, a class of the program, a function
	// of the module (inside functions), or a plain variable
	private Expression name(
			String name,
			ParserRuleContext ctx) {
		for (Map<String, String> scope : comprehensions)
			if (scope.containsKey(name))
				// a variable of a comprehension
				return new VariableRef(currentCFG, getLocation(filePath, ctx), scope.get(name));
		NativeCFG builtin = PyBuiltinRef.builtin(program, name);
		if (builtin != null)
			// e.g. q = len
			return new PyBuiltinRef(currentCFG, getLocation(filePath, ctx), builtin);
		Unit unit = program.getUnit(name);
		if (unit instanceof ClassUnit && ((ClassUnit) unit).getLocation() instanceof SourceCodeLocation)
			// a class defined in the program (e.g. MyClass in
			// check_type(a, MyClass))
			return new PyClassRef(currentCFG, getLocation(filePath, ctx), (ClassUnit) unit);
		LocalImports.Member member = localImports.member(name);
		if (member != null)
			// a function imported from a local module (from m import f)
			return new PyFunctionRef(currentCFG, getLocation(filePath, ctx), name, member.unit(), member.name());
		if (!currentCFG.getDescriptor().getName().equals(PyFileParser.INSTRUMENTED_MAIN_FUNCTION_NAME))
			// inside a function, it might be a function of the module
			return new PyFunctionRef(currentCFG, getLocation(filePath, ctx), name);
		return new VariableRef(currentCFG, getLocation(filePath, ctx), name);
	}

	// the literal of the number with the given text, at the location of ctx
	private Expression number(
			String literal,
			ParserRuleContext ctx) {
		String text = literal.toLowerCase().replaceAll("_", "");
		if (text.endsWith("j"))
			// complex number
			throw new UnsupportedStatementException(
					"complex numbers are not supported (at " + getLocation(filePath, ctx) + ")");

		if (text.contains("e") || text.contains("."))
			// floating point
			return new Float32Literal(currentCFG, getLocation(filePath, ctx), Float.parseFloat(text));

		// integer
		if (text.startsWith("0x"))
			return new Int32Literal(currentCFG, getLocation(filePath, ctx),
					Integer.parseInt(text.substring(2), 16));
		if (text.startsWith("0o"))
			return new Int32Literal(currentCFG, getLocation(filePath, ctx), Integer.parseInt(text.substring(2), 8));
		if (text.startsWith("0b"))
			return new Int32Literal(currentCFG, getLocation(filePath, ctx), Integer.parseInt(text.substring(2), 2));
		return new Int32Literal(currentCFG, getLocation(filePath, ctx), Integer.parseInt(text));
	}

	// the literal of the (adjacent) strings, at the location of ctx
	private Expression strings(
			StringsContext strings,
			ParserRuleContext ctx) {
		if (!strings.tstring().isEmpty())
			throw new UnsupportedStatementException("template strings are not supported");
		if (!strings.fstring().isEmpty())
			return visitFormattedStrings(strings);
		// adjacent literals are concatenated ("ab" "cd" == "abcd")
		StringBuilder value = new StringBuilder();
		int bytes = 0;
		for (StringContext literal : strings.string()) {
			value.append(PyStringLiterals.decode(literal.getText()));
			if (PyStringLiterals.isBytes(literal.getText()))
				bytes++;
		}
		if (bytes == 0)
			return new PyStringLiteral(currentCFG, getLocation(filePath, ctx), value.toString(),
					PyStringLiterals.quotes(strings.string(0).getText()));
		if (bytes != strings.string().size())
			throw new UnsupportedStatementException(
					"cannot mix bytes and nonbytes literals (at " + getLocation(filePath, ctx) + ")");
		if (value.chars().anyMatch(c -> c > 0xff))
			throw new UnsupportedStatementException(
					"bytes can only contain ASCII literal characters (at " + getLocation(filePath, ctx) + ")");
		return new PyBytesLiteral(currentCFG, getLocation(filePath, ctx), PyBytes.fromLatin1(value.toString()));
	}

	@Override
	public Expression visitAtom(
			AtomContext ctx) {
		if (ctx.name() != null)
			return name(ctx.name().getText(), ctx);
		else if (ctx.NUMBER() != null)
			return number(ctx.NUMBER().getText(), ctx);
		else if (ctx.FALSE() != null)
			return new FalseLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.TRUE() != null)
			return new TrueLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.NONE() != null)
			return new PyNoneLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.strings() != null)
			return strings(ctx.strings(), ctx);
		else if (ctx.tuple() != null) {
			TupleContext tuple = ctx.tuple();
			List<Expression> elements = new ArrayList<>();
			if (tuple.star_named_expression() != null)
				elements.add(visitStar_named_expression(tuple.star_named_expression()));
			elements.addAll(extractExpressionsFromStar_named_expressions(tuple.star_named_expressions()));
			TupleCreation tupleCreation = new TupleCreation(currentCFG, getLocation(filePath, ctx),
					elements.toArray(Expression[]::new));
			if (tupleCreation.getSubExpressions().length == 1)
				return tupleCreation.getSubExpressions()[0];
			return tupleCreation;
		} else if (ctx.group() != null) {
			if (ctx.group().named_expression() == null)
				throw new UnsupportedStatementException("yield expressions are not supported");
			return visitNamed_expression(ctx.group().named_expression());
		} else if (ctx.genexp() != null)
			throw new UnsupportedStatementException("generator expressions are not supported");
		else if (ctx.list() != null) {
			List<Expression> sts = extractExpressionsFromStar_named_expressions(ctx.list().star_named_expressions());
			return new ListCreation(currentCFG, getLocation(filePath, ctx), sts.toArray(Expression[]::new));
		} else if (ctx.listcomp() != null)
			return visitListComprehension(ctx.listcomp());
		else if (ctx.dict() != null) {
			List<Pair<Expression, Expression>> values = extractPairsFromDict(ctx.dict().double_starred_kvpairs());
			DictionaryCreation r = new DictionaryCreation(currentCFG, getLocation(filePath, ctx),
					values.toArray(Pair[]::new));
			return r;
		} else if (ctx.set() != null) {
			List<Expression> values = extractExpressionsFromStar_named_expressions(ctx.set().star_named_expressions());
			return new SetCreation(currentCFG, getLocation(filePath, ctx), values.toArray(Expression[]::new));
		} else if (ctx.dictcomp() != null || ctx.setcomp() != null)
			throw new UnsupportedStatementException("comprehensions are not supported");
		else if (ctx.ELLIPSIS() != null)
			throw new UnsupportedStatementException();
		throw new UnsupportedStatementException();
	}

	public AnnotationMember visitDecorator(
			Named_expressionContext ctx) {
		Expression expr = visitNamed_expression(ctx);

		List<Expression> params = new ArrayList<>();
		UnresolvedCall uc;
		if (expr instanceof UnresolvedCall) {
			uc = (UnresolvedCall) expr;
			params.addAll(Arrays.asList(uc.getParameters()));
		} else {
			params.add(expr);
			uc = new UnresolvedCall(currentCFG, getLocation(filePath, ctx), CallType.UNKNOWN, null, "",
					params.toArray(Expression[]::new));
		}
		return new AnnotationMember(ctx.getText(), new DecoratedAnnotation(params, uc));
	}

	@Override
	public Annotation visitDecorators(
			DecoratorsContext ctx) {
		List<AnnotationMember> annotationMembers = new ArrayList<>();
		for (Named_expressionContext dc : ctx.named_expression()) {
			AnnotationMember am = visitDecorator(dc);
			annotationMembers.add(am);
		}
		Annotation annotation = new Annotation("$decorators", annotationMembers);
		return annotation;
	}

	@Override
	public ClassUnit visitClass_def(
			Class_defContext ctx) {
		if (ctx.decorators() != null)
			throw new UnsupportedStatementException("decorators are not supported");
		PyClassParser parser = new PyClassParser(program, filePath, localImports);
		return parser.visitClass_def_raw(ctx.class_def_raw());
	}

	@Override
	public PyCFG visitFunction_def(
			Function_defContext ctx) {
		if (ctx.decorators() != null)
			throw new UnsupportedStatementException("decorators are not supported");
		PyCodeMemberParser parser = new PyCodeMemberParser(filePath, program, currentUnit, localImports);
		return parser.visitFunction_def_raw(ctx.function_def_raw());
	}
}
