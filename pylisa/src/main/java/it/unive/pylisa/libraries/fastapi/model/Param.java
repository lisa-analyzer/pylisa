package it.unive.pylisa.libraries.fastapi.model;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.List;

@Getter
@Setter
@NoArgsConstructor
public class Param {
    private String type;
    private String name;

    public Param(String typeHint, String name) {

        switch(typeHint) {
            case "str":
                this.type = "string";
                break;
            case "bool":
                this.type = "boolean";
                break;
            case "int":
                this.type = "numeric";
                break;
            default:
                this.type = typeHint;
        }

        this.name = name;
    }

    public Boolean isTypeCustomDefinition() {
        return !List.of("string", "boolean", "numeric").contains(this.type);
    }
}
