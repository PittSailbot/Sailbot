## Welcome to Pitt Sailbot!
Howdy! 

## Contributing

Check out the [Getting Started](Getting Started.md) for steps on how to set up your environment to contribute.

## Resources

Commonly used resources and commands can be found under the Tips tab.

## How to edit this site
Documentation is written using markdown and published through Github pages using mkdocs. If you're unfamiliar with using markdown, check out this handy [cheat sheet](https://www.markdownguide.org/cheat-sheet/).

### How to make changes
In depth documentation for mkdocs can be found [here](https://www.mkdocs.org/user-guide/writing-your-docs/). 

1. In the terminal, install mkdocs and the mkdocs material theme with:
```console
pip install mkdocs
```
```console
pip install mkdocs-material
```
2. If adding any new files, add the file under the mkdocs.yml's nav config like so:
```
nav:
   - Getting Started: "Getting Started.md"
```

This will make the file "Getting Started.md" visible on the web page's index

### Previewing changes
To preview the documentation site before pushing changes to github:
1. In the terminal, run:
```console
mkdocs serve
```
2. In any web browser navigate to 127.0.0.1:8000