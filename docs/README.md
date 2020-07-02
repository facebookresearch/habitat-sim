## Building Docs Locally

```bash
python setup.py install
cd docs
conda install -y -c conda-forge doxygen==1.8.16
conda install -y  jinja2 pygments docutils
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
../build/docs-public/habitat-cpp/index.html
../build/docs-public/habitat-sim/index.html
```
