package it.unive.pylisa;

public class UnsupportedStatementException extends RuntimeException {
    public UnsupportedStatementException() {
        super();
    }
    public UnsupportedStatementException(String s) {
        super(s);
    }
    public UnsupportedStatementException(String s, Exception e) {
        super(s, e);
    }
}
