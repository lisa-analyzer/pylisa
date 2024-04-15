package it.unive.pylisa.libraries.fastapi.models;

import lombok.Getter;

@Getter
public enum Method {

    GET("get"),
    POST("post"),
    PUT("put"),
    DELETE("delete");

    private final String value;

    Method(String method) {
        this.value = method;
    }

    public static Method specify(String decoratorID) {

        for (Method method : Method.values()) {
            if (decoratorID.contains(method.getValue())) {
                return method;
            }
        }

        return null;
    }
}
