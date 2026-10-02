package it.unive.pylisa.frontend.expression;

/**
 * Python dunder (double-underscore) method names used by the front-end
 * lowering.
 */
public final class DunderMethods {

	public static final String GETITEM = "__getitem__";
	public static final String SETITEM = "__setitem__";
	public static final String DELITEM = "__delitem__";
	public static final String LEN = "__len__";
	public static final String LT = "__lt__";
	public static final String LE = "__le__";
	public static final String GT = "__gt__";
	public static final String GE = "__ge__";
	public static final String EQ = "__eq__";
	public static final String NE = "__ne__";
	public static final String ITER = "__iter__";
	public static final String NEXT = "__next__";
	public static final String CONTAINS = "__contains__";
	public static final String INIT = "__init__";

	private DunderMethods() {
	}
}
