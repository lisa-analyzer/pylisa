package it.unive.pylisa.frontend;

import it.unive.pylisa.antlr.Python3Parser.ClassdefContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_forContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Except_clauseContext;
import it.unive.pylisa.antlr.Python3Parser.Expr_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.ExprlistContext;
import it.unive.pylisa.antlr.Python3Parser.For_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.Global_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_fromContext;
import it.unive.pylisa.antlr.Python3Parser.Namedexpr_testContext;
import it.unive.pylisa.antlr.Python3Parser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.TfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.With_itemContext;
import java.util.HashSet;
import java.util.Set;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;

/**
 * The names a source file may bind anywhere, in any scope: definitions,
 * parameters, imports, targets of assignments, of {@code for} and
 * {@code with}, of {@code except ... as}, of {@code :=} and of
 * comprehensions, and names declared {@code global} or {@code nonlocal}. A
 * file with a star import may bind any name.
 * <p>
 * The set over-approximates: a name bound anywhere counts as bound everywhere
 * in the file. It decides whether a name of the builtins certainly refers to
 * the builtin, and whether the name of a base class certainly refers to the
 * class it was resolved to.
 * </p>
 */
public final class BoundNames {

	private final Set<String> names = new HashSet<>();

	/**
	 * The names bound otherwise than by a {@code class} definition or an
	 * import.
	 */
	private final Set<String> otherwise = new HashSet<>();

	private final Set<String> classes = new HashSet<>();

	private final Set<String> imported = new HashSet<>();

	private boolean starImport;

	private BoundNames() {
	}

	/**
	 * Collects the names bound by a file.
	 *
	 * @param file the parse tree of the whole file
	 *
	 * @return the names
	 */
	public static BoundNames of(
			ParserRuleContext file) {
		BoundNames bound = new BoundNames();
		bound.collect(file);
		return bound;
	}

	/**
	 * Yields whether the file may bind a name.
	 *
	 * @param name the name
	 *
	 * @return {@code true} if some construct of the file may bind it
	 */
	public boolean mayBind(
			String name) {
		return starImport || names.contains(name);
	}

	/**
	 * Yields whether the file may bind a name otherwise than by one of its
	 * {@code class} definitions or one of its imports, so that the name may
	 * not refer to the class those give it.
	 *
	 * @param name the name
	 *
	 * @return {@code true} if it may
	 */
	public boolean mayRebind(
			String name) {
		// a class of the file and an import of the same name: either may be
		// the one in scope
		return starImport || otherwise.contains(name) || classes.contains(name) && imported.contains(name);
	}

	private void collect(
			ParseTree tree) {
		if (tree instanceof Import_fromContext from)
			for (int i = 0; i < from.getChildCount(); i++)
				starImport |= from.getChild(i).getText().equals("*");
		if (tree instanceof TerminalNode terminal) {
			if (binds(terminal)) {
				names.add(terminal.getText());
				ParseTree parent = terminal.getParent();
				if (parent instanceof ClassdefContext)
					classes.add(terminal.getText());
				else if (parent instanceof Import_as_nameContext || parent instanceof Dotted_as_nameContext
						|| parent instanceof Dotted_nameContext && parent.getParent() instanceof Dotted_as_nameContext)
					imported.add(terminal.getText());
				if (!(parent instanceof ClassdefContext || parent instanceof Import_as_nameContext
						|| parent instanceof Dotted_nameContext && parent.getParent() instanceof Dotted_as_nameContext
						|| parent instanceof Dotted_as_nameContext))
					otherwise.add(terminal.getText());
			}
			return;
		}
		for (int i = 0; i < tree.getChildCount(); i++)
			collect(tree.getChild(i));
	}

	private static boolean binds(
			TerminalNode name) {
		ParseTree child = name;
		ParseTree parent = name.getParent();
		if (parent instanceof FuncdefContext || parent instanceof ClassdefContext || parent instanceof TfpdefContext
				|| parent instanceof VfpdefContext || parent instanceof Import_as_nameContext
				|| parent instanceof Global_stmtContext || parent instanceof Nonlocal_stmtContext
				|| parent instanceof Except_clauseContext)
			// for except, only the name after "as" is a direct terminal child
			return true;
		while (parent != null) {
			if (parent instanceof Dotted_as_nameContext)
				return true;
			if (parent instanceof ExprlistContext
					&& (parent.getParent() instanceof For_stmtContext || parent.getParent() instanceof Comp_forContext))
				return true;
			if (parent instanceof With_itemContext with && with.expr() == child)
				return true;
			if (parent instanceof Namedexpr_testContext walrus && walrus.test().size() > 1 && walrus.test(0) == child)
				return true;
			if (parent instanceof Expr_stmtContext statement)
				return isTarget(statement, child);
			child = parent;
			parent = parent.getParent();
		}
		return false;
	}

	/**
	 * Yields whether a child of an expression statement is assigned to: every
	 * part before the last {@code =}, or the target of an augmented or
	 * annotated assignment.
	 */
	private static boolean isTarget(
			Expr_stmtContext statement,
			ParseTree child) {
		if (statement.augassign() != null || statement.annassign() != null)
			return statement.getChild(0) == child;
		int last = -1;
		for (int i = 0; i < statement.getChildCount(); i++)
			if (statement.getChild(i).getText().equals("="))
				last = i;
		for (int i = 0; i < last; i++)
			if (statement.getChild(i) == child)
				return true;
		return false;
	}
}
