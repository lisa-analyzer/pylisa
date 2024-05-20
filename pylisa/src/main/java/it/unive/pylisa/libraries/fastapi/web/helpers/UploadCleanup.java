package it.unive.pylisa.libraries.fastapi.web.helpers;

import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.core.io.Resource;
import org.springframework.core.io.ResourceLoader;
import org.springframework.stereotype.Component;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

@Component
public class UploadCleanup implements ApplicationRunner {

    private final ResourceLoader resourceLoader;

    public UploadCleanup(ResourceLoader resourceLoader) {
        this.resourceLoader = resourceLoader;
    }

    @Override
    public void run(ApplicationArguments args) throws Exception {
        cleanUploadFolder();
    }

    private void cleanUploadFolder() throws IOException {
        Resource resource = resourceLoader.getResource("file:pylisa/py-testcases/microservices/uploded-from-endpoint/");
        File uploadFolder = resource.getFile();

        if (uploadFolder.exists() && uploadFolder.isDirectory()) {
            Files.list(uploadFolder.toPath())
                    .forEach(path -> {
                        try {
                            Files.delete(path);
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    });
        }
    }
}
