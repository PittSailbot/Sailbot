Documentation is written using markdown and published through Github pages using mkdocs. If you're unfamiliar with using markdown, check out this handy [cheat sheet](https://www.markdownguide.org/cheat-sheet/).

# Making simple additions/changes
This covers pretty much all changes you would ever need to make
1. Checkout the [/docs branch](https://github.com/PittSailbot/Sailbot/tree/docs)
2. Edit or add new files
   - If adding any new files, add the file under the mkdocs.yml's nav config like so:
    ```
    nav:
       - Getting Started: "Getting Started.md"
    ```
3. Commit and push

# Making major/thematic changes
If you want to change how the documentation looks or want features from external plugins then you'll need to install mkdocs to serve a local page so we can preview what we're doing.

In depth documentation for mkdocs can be found [here](https://www.mkdocs.org/user-guide/writing-your-docs/). 

1. First, in the terminal, install mkdocs and the mkdocs material theme with:
```console
pip install mkdocs
```
```console
pip install mkdocs-material
```


This will make the file "Getting Started.md" visible on the web page's index

## Previewing changes
To preview the documentation site before pushing changes to github:
1. In the terminal, run:
```console
mkdocs serve
```
2. In any web browser navigate to 127.0.0.1:8000
