## Building Docs Locally

```bash
python setup.py install
cd docs
git submodule update --init
./build.sh # or ./build-public.sh when deploying to aihabitat.org
```

## Viewing Docs Locally

After a successful build, the `build.sh` script will show clickable links
pointing to the output. The output for C++ and Python docs is located at:

```
../build/docs-public/habitat-sim/index.html
```
