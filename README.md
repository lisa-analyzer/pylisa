# PyLiSA

Python frontend for LiSA

## Building with snapshots

Sometimes, PyLiSA will refer to a snapshot release of LiSA to exploit unreleased features. If, when building, you get the following error message:

```
> Could not resolve com.github.unive-ssv:lisa-project:ver-SNAPSHOT.
  > Could not get resource 'https://maven.pkg.github.com/UniVE-SSV/lisa/com/github/unive-ssv/lisa-project/ver-SNAPSHOT/maven-metadata.xml'.
    > Could not GET 'https://maven.pkg.github.com/UniVE-SSV/lisa/com/github/unive-ssv/lisa-project/ver-SNAPSHOT/maven-metadata.xml'. Received status code 401 from server: Unauthorized
```

Then you have to perform the following:
- Create a GitHub PAT following [this guide](https://docs.github.com/en/enterprise-cloud@latest/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) and grant `read:packages` permission
- Create a `gradle.properties` file in the project root folder (where the `gradlew` scripts are located) with the following content:
```
gpr.user=your-github-username
gpr.key=github-access-token
```

Then, re-execute the build to have the snapshot dependencies downloaded.
