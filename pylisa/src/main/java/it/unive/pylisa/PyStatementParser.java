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
import it.unive.lisa.program.cfg.statement.Ret;
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
import it.unive.lisa.program.cfg.statement.logic.Not;
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
import it.unive.pylisa.antlr.PythonParser.Class_defContext;
import it.unive.pylisa.antlr.PythonParser.Compare_op_bitwise_or_pairContext;
import it.unive.pylisa.antlr.PythonParser.ComparisonContext;
import it.unive.pylisa.antlr.PythonParser.Compound_stmtContext;
import it.unive.pylisa.antlr.PythonParser.ConjunctionContext;
import it.unive.pylisa.antlr.PythonParser.Continue_stmtContext;
import it.unive.pylisa.antlr.PythonParser.DecoratorsContext;
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
import it.unive.pylisa.antlr.PythonParser.For_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Function_defContext;
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
import it.unive.pylisa.antlr.PythonParser.LambdefContext;
import it.unive.pylisa.antlr.PythonParser.Named_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Pass_stmtContext;
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
import it.unive.pylisa.antlr.PythonParser.SumContext;
import it.unive.pylisa.antlr.PythonParser.T_primaryContext;
import it.unive.pylisa.antlr.PythonParser.Target_with_star_atomContext;
import it.unive.pylisa.antlr.PythonParser.TermContext;
import it.unive.pylisa.antlr.PythonParser.Try_stmtContext;
import it.unive.pylisa.antlr.PythonParser.TupleContext;
import it.unive.pylisa.antlr.PythonParser.While_stmtContext;
import it.unive.pylisa.antlr.PythonParser.With_itemContext;
import it.unive.pylisa.antlr.PythonParser.With_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Yield_exprContext;
import it.unive.pylisa.antlr.PythonParser.Yield_stmtContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAccessInstanceGlobal;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyBitwiseAnd;
import it.unive.pylisa.cfg.expression.PyBitwiseLeftShift;
import it.unive.pylisa.cfg.expression.PyBitwiseNot;
import it.unive.pylisa.cfg.expression.PyBitwiseOr;
import it.unive.pylisa.cfg.expression.PyBitwiseRIghtShift;
import it.unive.pylisa.cfg.expression.PyBitwiseXor;
import it.unive.pylisa.cfg.expression.PyDivision;
import it.unive.pylisa.cfg.expression.PyDoubleArrayAccess;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyMethodCall;
import it.unive.pylisa.cfg.expression.PyMultiplication;
import it.unive.pylisa.cfg.expression.PyNegation;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.PySingleArrayAccess;
import it.unive.pylisa.cfg.expression.PySubtraction;
import it.unive.pylisa.cfg.expression.PyTernaryOperator;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
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
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.expression.literal.PyBytesLiteral;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import it.unive.pylisa.cfg.expression.literal.PyStringLiterals;
import it.unive.pylisa.cfg.expression.literal.PyTypeLiteral;
import it.unive.pylisa.cfg.expression.unary.PyLength;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.statement.SimpleSuperUnresolvedCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.NoOpFunction;
import it.unive.pylisa.symbolic.PyBytes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;
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
			ControlFlowTracker control) {
		this.program = program;
		this.filePath = filePath;
		this.currentUnit = currentUnit;
		this.currentCFG = currentCFG;
		this.control = control;
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
		if (ctx.assignment() != null)
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
		if (ctx.augassign() != null) {
			// x op= y ~> x = x op y
			// TODO: for a subscript/attribute target (e.g. container[i] += y),
			// this evaluates the target's base expression (container, i) twice
			// instead of once, unlike real Python. Harmless for the currently
			// imprecise Sequence/attribute domains, but not exact semantics.
			Expression rhs = visitAnnotated_rhs(ctx.annotated_rhs());
			Expression writeTarget = visitSingleTarget(ctx.single_target());
			Expression readTarget = visitSingleTarget(ctx.single_target());
			Expression op = buildAugmentedOp(ctx.augassign(), getLocation(filePath, ctx), readTarget, rhs);
			return new PyAssign(currentCFG, getLocation(filePath, ctx), writeTarget, op);
		}

		Expression value = visitAnnotated_rhs(ctx.annotated_rhs());
		List<Star_targetsContext> targets = ctx.star_targets();
		for (int i = targets.size() - 1; i >= 0; i--)
			value = new PyAssign(currentCFG, getLocation(filePath, ctx), visitStar_targets(targets.get(i)), value);
		return value;
	}

	private Expression visitSingleTarget(
			Single_targetContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		if (ctx.single_target() != null)
			return visitSingleTarget(ctx.single_target());
		return visitSingleSubscriptAttributeTarget(ctx.single_subscript_attribute_target());
	}

	private Expression visitSingleSubscriptAttributeTarget(
			Single_subscript_attribute_targetContext ctx) {
		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return new UnresolvedCall(
					currentCFG,
					getLocation(filePath, ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(filePath, ctx), ctx.name().getText(), "'"));

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

	private Expression buildAugmentedOp(
			AugassignContext ctx,
			SourceCodeLocation loc,
			Expression left,
			Expression right) {
		if (ctx.PLUSEQUAL() != null)
			return new PyAddition(currentCFG, loc, left, right);
		if (ctx.MINEQUAL() != null)
			return new PySubtraction(currentCFG, loc, left, right);
		if (ctx.STAREQUAL() != null)
			return new PyMultiplication(currentCFG, loc, left, right);
		if (ctx.ATEQUAL() != null)
			return new PyMatMul(currentCFG, loc, left, right);
		if (ctx.SLASHEQUAL() != null)
			return new PyDivision(currentCFG, loc, left, right);
		if (ctx.PERCENTEQUAL() != null)
			return new PyRemainder(currentCFG, loc, left, right);
		if (ctx.AMPEREQUAL() != null)
			return new PyBitwiseAnd(currentCFG, loc, left, right);
		if (ctx.VBAREQUAL() != null)
			return new PyBitwiseOr(currentCFG, loc, left, right);
		if (ctx.CIRCUMFLEXEQUAL() != null)
			return new PyBitwiseXor(currentCFG, loc, left, right);
		if (ctx.LEFTSHIFTEQUAL() != null)
			return new PyBitwiseLeftShift(currentCFG, loc, left, right);
		if (ctx.RIGHTSHIFTEQUAL() != null)
			return new PyBitwiseRIghtShift(currentCFG, loc, left, right);
		if (ctx.DOUBLESTAREQUAL() != null)
			return new PyPower(currentCFG, loc, left, right);
		if (ctx.DOUBLESLASHEQUAL() != null)
			return new PyFloorDiv(currentCFG, loc, left, right);
		throw new UnsupportedStatementException("Unknown augmented assignment operator");
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
			return new UnresolvedCall(
					currentCFG,
					getLocation(filePath, ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(filePath, ctx), ctx.name().getText(), "'"));

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
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		if (ctx.target_with_star_atom() != null)
			return visitTargetWithStarAtom(ctx.target_with_star_atom());
		throw new UnsupportedStatementException("Tuple/list unpacking targets are not supported");
	}

	private Expression visitTPrimary(
			T_primaryContext ctx) {
		if (ctx.t_primary() == null)
			return visitAtom(ctx.atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return new UnresolvedCall(
					currentCFG,
					getLocation(filePath, ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(filePath, ctx), ctx.name().getText(), "'"));
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
			return new UnresolvedCall(
					currentCFG,
					getLocation(filePath, ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(filePath, ctx), ctx.name().getText(), "'"));

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

	private Expression visitDelTAtom(
			Del_t_atomContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
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
			return new Ret(currentCFG, getLocation(filePath, ctx));
		return new Return(currentCFG, getLocation(filePath, ctx), visitStar_expressions(ctx.star_expressions()));
	}

	@Override
	public Statement visitYield_stmt(
			Yield_stmtContext ctx) {
		List<Expression> l = extractYieldArguments(ctx.yield_expr());
		return new UnresolvedCall(
				currentCFG,
				getLocation(filePath, ctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"yield from",
				LeftToRightEvaluation.INSTANCE,
				l.toArray(new Expression[0]));
	}

	@Override
	public Statement visitRaise_stmt(
			Raise_stmtContext ctx) {
		log.warn("Exceptions are not yet supported. The raise statement at line " + getLine(ctx) + " of file "
				+ filePath + " is unsoundly translated into a return; statement");
		return new Ret(currentCFG, getLocation(filePath, ctx));
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
	public Expression visitAssert_stmt(
			Assert_stmtContext ctx) {
		List<Expression> args = new ArrayList<>();
		for (ExpressionContext e : ctx.expression())
			args.add(visitExpression(e));
		return new UnresolvedCall(
				currentCFG,
				getLocation(filePath, ctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				args.toArray(Expression[]::new));
	}

	@Override
	public ParsedBlock visitCompound_stmt(
			Compound_stmtContext ctx) {
		if (ctx.function_def() != null) {
			PyCFG fun = visitFunction_def(ctx.function_def());
			if (currentUnit instanceof ClassUnit)
				((ClassUnit) currentUnit).addInstanceCodeMember(fun);
			else
				currentUnit.addCodeMember(fun);
			// the definition itself has no runtime effect in the enclosing
			// CFG: the function is registered as a code member, not inlined
			return noOpBlock(ctx);
		} else if (ctx.class_def() != null) {
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
			throw new UnsupportedStatementException("match statements are not supported");
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

	@Override
	public ParsedBlock visitWhile_stmt(
			While_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of while
		NoOp whileExitNode = new NoOp(currentCFG, getLocation(filePath, ctx));
		block.addNode(whileExitNode);

		Statement condition = visitNamed_expression(ctx.named_expression());
		block.addNode(condition);

		ParsedBlock trueBlock = visitBlock(ctx.block());

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

		VariableRef counter = new VariableRef(
				currentCFG,
				getLocation(filePath, ctx),
				"__counter_location" + getLocation(filePath, ctx).getLine(), Int32Type.INSTANCE);
		Expression[] counter_pars = { collection, counter };

		// counter = 0;
		Assignment counter_init = new Assignment(
				currentCFG,
				getLocation(filePath, ctx),
				counter,
				new Int32Literal(currentCFG, getLocation(filePath, ctx), 0));
		block.addNode(counter_init);

		// counter < collection.size()
		UnresolvedCall condition = new UnresolvedCall(
				currentCFG,
				getLocation(filePath, ctx),
				CallType.STATIC,
				null,
				"__lt__",
				counter,
				new UnresolvedCall(
						currentCFG,
						getLocation(filePath, ctx),
						CallType.STATIC,
						null,
						"__len__",
						LeftToRightEvaluation.INSTANCE,
						collection));
		block.addNode(condition);

		// element = collection.at(counter)
		Assignment element_assignment = new Assignment(
				currentCFG,
				getLocation(filePath, ctx),
				variable,
				new UnresolvedCall(
						currentCFG,
						getLocation(filePath, ctx),
						CallType.STATIC,
						null,
						"__getitem__",
						LeftToRightEvaluation.INSTANCE,
						counter_pars));
		block.addNode(element_assignment);

		// counter = counter + 1;
		Assignment counter_increment = new Assignment(
				currentCFG,
				getLocation(filePath, ctx),
				counter,
				new PyAddition(
						currentCFG,
						getLocation(filePath, ctx),
						counter,
						new Int32Literal(
								currentCFG,
								getLocation(filePath, ctx),
								1)));
		block.addNode(counter_increment);

		ParsedBlock body = visitBlock(ctx.block());
		block.mergeWith(body.getBody());
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
		return new ParsedBlock(counter_init, block, exit);
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

	@Override
	public Expression visitLambdef(
			LambdefContext ctx) {
		if (ctx.lambda_params() != null)
			throw new UnsupportedStatementException("lambda parameters are not supported");
		Expression body = visitExpression(ctx.expression());
		return new LambdaExpression(
				new ArrayList<Expression>(),
				body,
				currentCFG,
				getLocation(filePath, ctx));
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
			return new Not(currentCFG, getLocation(filePath, ctx), visitInversion(ctx.inversion()));
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
			return new Not(currentCFG, getLocation(filePath, ctx),
					new PyIn(currentCFG, getLocation(filePath, ctx), left,
							visitBitwise_or(ctx.notin_bitwise_or().bitwise_or())));
		else if (ctx.in_bitwise_or() != null)
			return new PyIn(currentCFG, getLocation(filePath, ctx), left,
					visitBitwise_or(ctx.in_bitwise_or().bitwise_or()));
		else if (ctx.isnot_bitwise_or() != null)
			return new Not(currentCFG, getLocation(filePath, ctx),
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
				access = new UnresolvedCall(
						currentCFG,
						getLocation(filePath, frame),
						CallType.INSTANCE,
						null,
						"__getattribute__",
						access,
						new PyStringLiteral(currentCFG, getLocation(filePath, frame), last_name, "'"));
			} else if (frame.LPAR() != null) {
				if (last_name == null)
					return new Empty(currentCFG, getLocation(filePath, frame));

				List<Expression> pars = extractArguments(frame.arguments());
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
				} else if (!instance && method_name.equals("len") && pars.size() == 1) {
					access = new PyLength(currentCFG, getLocation(filePath, frame), pars.get(0));
				} else {
					access = instance
							? new PyMethodCall(
									currentCFG,
									getLocation(filePath, frame),
									method_name,
									LeftToRightEvaluation.INSTANCE,
									pars.toArray(Expression[]::new))
							: new UnresolvedCall(
									currentCFG,
									getLocation(filePath, frame),
									CallType.STATIC,
									// e.g. bytes.fromhex(s) is looked up in
									// Bytes
									attribute && previous_access instanceof VariableRef
											? BUILTIN_CLASSES.get(((VariableRef) previous_access).getName())
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
					return NoOpFunction.build(currentCFG, getLocation(filePath, ctx), null);
			} else if (frame.genexp() != null)
				throw new UnsupportedStatementException("generator expression calls are not supported");
			else
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
	 * Whether {@code expr} denotes a namespace rather than a value: a name
	 * bound by an import, the name of a class or of a library, or an attribute
	 * of one of those (e.g. {@code os.path}).
	 */
	private boolean isNamespace(
			Expression expr) {
		if (expr instanceof VariableRef) {
			String name = ((VariableRef) expr).getName();
			return namespaces.contains(name) || program.getUnit(name) != null || BUILTIN_CLASSES.containsKey(name);
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
				result.add(visitStarred_expression((Starred_expressionContext) child));
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
		return visitStarred_expression(ctx.starred_expression());
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
		Expression start = new Empty(currentCFG, loc);
		Expression stop = new Empty(currentCFG, loc);
		Expression step = new Empty(currentCFG, loc);
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
				throw new UnsupportedStatementException("** dict unpacking is not supported");
			result.add(Pair.of(visitExpression(e.kvpair().expression(0)), visitExpression(e.kvpair().expression(1))));
		}
		return result;
	}

	@Override
	public Expression visitAtom(
			AtomContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(filePath, ctx), ctx.name().getText());
		else if (ctx.NUMBER() != null) {
			String text = ctx.NUMBER().getText().toLowerCase().replaceAll("_", "");
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
		} else if (ctx.FALSE() != null)
			return new FalseLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.TRUE() != null)
			return new TrueLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.NONE() != null)
			return new PyNoneLiteral(currentCFG, getLocation(filePath, ctx));
		else if (ctx.strings() != null) {
			if (!ctx.strings().fstring().isEmpty() || !ctx.strings().tstring().isEmpty()
					|| ctx.strings().string().isEmpty())
				throw new UnsupportedStatementException("formatted strings are not supported");
			// adjacent literals are concatenated ("ab" "cd" == "abcd")
			StringBuilder value = new StringBuilder();
			int bytes = 0;
			for (StringContext literal : ctx.strings().string()) {
				value.append(PyStringLiterals.decode(literal.getText()));
				if (PyStringLiterals.isBytes(literal.getText()))
					bytes++;
			}
			if (bytes == 0)
				return new PyStringLiteral(currentCFG, getLocation(filePath, ctx), value.toString(),
						PyStringLiterals.quotes(ctx.strings().string(0).getText()));
			if (bytes != ctx.strings().string().size())
				throw new UnsupportedStatementException(
						"cannot mix bytes and nonbytes literals (at " + getLocation(filePath, ctx) + ")");
			if (value.chars().anyMatch(c -> c > 0xff))
				throw new UnsupportedStatementException(
						"bytes can only contain ASCII literal characters (at " + getLocation(filePath, ctx) + ")");
			return new PyBytesLiteral(currentCFG, getLocation(filePath, ctx), PyBytes.fromLatin1(value.toString()));
		} else if (ctx.tuple() != null) {
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
			throw new UnsupportedStatementException("list comprehensions are not supported");
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

	private List<Expression> extractYieldArguments(
			Yield_exprContext ctx) {
		List<Expression> r = new ArrayList<>(1);
		if (ctx.FROM() != null)
			r.add(visitExpression(ctx.expression()));
		else if (ctx.star_expressions() != null)
			r.add(visitStar_expressions(ctx.star_expressions()));
		return r;
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
		PyClassParser parser = new PyClassParser(program, filePath);
		return parser.visitClass_def_raw(ctx.class_def_raw());
	}

	@Override
	public PyCFG visitFunction_def(
			Function_defContext ctx) {
		if (ctx.decorators() != null)
			throw new UnsupportedStatementException("decorators are not supported");
		PyCodeMemberParser parser = new PyCodeMemberParser(filePath, program, currentUnit);
		return parser.visitFunction_def_raw(ctx.function_def_raw());
	}
}
