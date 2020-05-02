## Building Docs Locally

```bash
python setup.py install (#or ./build.sh) 
cd docs
mkdir -p ../build/docs
git submodule update --init
./build-public.sh
```

## Viewing Docs Locally
../build/docs-public/habitat-cpp/index.html
../build/docs-public/habitat-sim/index.html
