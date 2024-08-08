## Building Docs Locally
### First Time Setup:
Install required packages:
```
conda install -c conda-forge doxygen==1.8.16
conda install jinja2 pygments docutils
```

To build the docs, you will also need to install the following
packages on Linux :
```
sudo apt install --allow-change-held-packages \
                  texlive-base \
                  texlive-latex-extra \
                  texlive-fonts-extra \
                  texlive-fonts-recommended
```
or on OSX :
```
brew install --cask mactex
```
__Note__: You might also need to run the following command
as suggested by [this page](https://mcss.mosra.cz/plugins/math-and-code/#math)
```
export PATH=$PATH:/Library/TeX/Distributions/.DefaultTeX/Contents/Programs/texbin
```

### Building Docs:
```bash
python setup.py build_ext --inplace
cd docs
git submodule update --init
./build.sh # or ./build-public.sh when deploying to aihabitat.org
```

If you're having trouble with doxygen, check your version with

```bash
doxygen -v
```

and make sure you're using 1.8.16.

## Viewing Docs Locally

After a successful build, the `build.sh` script will show clickable links
pointing to the output. The output for C++ and Python docs is located at:

```
../build/docs-public/habitat-sim/index.html
```

## Docs developer guide

### Doxygen tag files

For Corrade and Magnum, up-to-date tagfiles can be found at
https://magnum.graphics/doc-downloads/; STL tagfile is linked from there as
well. Update the `docs/corrade.tag`, `docs/magnum.tag` and `docs/stl.tag` with
those.

For Bullet, the upstream doesn't generate or distribute a tagfile, so it's
needed to build it manually. The current one is generated from version 2.88 and
was created like this:

```
git clone https://github.com/bulletphysics/bullet3
cd bullet3
git checkout 2.88
nano Doxyfile # modify it to have GENERATE_TAGFILE = bullet.tag
doxygen # it'll complain about some CHM file, ignore that
cp bullet.tag ../habitat-sim/docs/
```

### Updating the CSS theme, refreshing the compiled version of it

Make your changes to `theme.css`. Easiest is to experiment in a browser
inspector and then copy the updated colors over to the theme file. Note that
the color variables are shared by multiple places, so for example changing
colors of the warning note will also affect labels and table backgrounds.

Then, run `./build.sh` (which regenerates `theme-compiled.css`, which is
deliberately *not* tracked in Git) and verify the generated output in the
browser again.

The `theme.css` should then get synced with the `habitat-lab` and
`habitat-website` repositories so it's always possible to play with the colors
in context of one repository and sync it to the others.

### Docstring Formatting

** Technical details: **
HTMLs are rendered when m.css parses doctrings.
Habitat-lab points to the habitat-sim repo, in habitat-sim/build/docs, where style options and .css files are rendered through m.css. The HTML configuration options are in conf.py file.  The option "M_SPHINX_PARSE_DOCSTRINGS=True" parses python docstrings.

Docstrings have the following general hierarchy:


```
First paragraph of module detailed docs."""

class Foo:
    """Class summary"""

    def bar(self):
        """Function summary"""
```


Doctrings follow reStructuredText from docutils library.


- Skip line for paragraph break

- Indent to indent

- "-" for unordered lists

- ">>>" lines of code

- For boldfont, surround text with "**"  "\*\*\<TEXT>\*\*", 


```
Heading 1
#########

Heading 2
=========

Heading 3
---------
```

**Directive options:**

NOTE, Be aware of spaces, and backtick should be used, not single quote

- :param \<VARIABLE NAME>:\<VARIABLE NAME DESCRIPTION>
  - Should be used for parameter descriptions

- :data \<VARIABLE NAME>: \<VARIABLE NAME VALUE>
  - Should be used for class parameters, ENUMS

- :return: \<DESCRIPTION OF FUNCTION OUTPUT>
  - Used at function level

- :py: \`\<PYTHON CODE VARIBLE>\`
  - Reformat string into Python code format

- :ref: \<MODDULE>.\<MODULE>.[...]\<CLASS>
  - Make a hyperlink that references to other parts of documentation

- :raises ExceptionType: Some multi-line \
        exception description.

- :math: \`\$\<LATEX MATH FUNCTION>\$`
  - This if meant to display math functions inline using LaTeX notation. NOTE:  use double slash \\\\ instead of \\ for special LaTeX functions.

- :property \<NAME OF PROPERTY>:  \<DESCRIPTION OF PROPERTY>
	- Between a class function and a class method. Usually for publically accessible setters and getters
	
References to more syntax examples and options:

https://mcss.mosra.cz/documentation/python/
https://docutils.sourceforge.io/docs/ref/rst/restructuredtext.html#quick-syntax-overview
