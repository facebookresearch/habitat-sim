#if !defined(__NVRTC_HELPER__)

#define __NVRTC_HELPER__ 1

#include <cuda.h>
#include <nvrtc.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <helper_cuda_drvapi.h>

#define NVRTC_SAFE_CALL(Name, x)                                             \
  do {                                                                       \
    nvrtcResult result = x;                                                  \
    if (result != NVRTC_SUCCESS) {                                           \
      std::cerr << "\nerror: " << Name << " failed with error " <<           \
                                               nvrtcGetErrorString(result);  \
      exit(1);                                                               \
    }                                                                        \
  } while(0)

void compileFileToPTX(char *filename, int argc, const char **argv,
                      char **ptxResult, size_t *ptxResultSize)
{
    std::ifstream inputFile(filename, std::ios::in | std::ios::binary |
                                std::ios::ate);

    if (!inputFile.is_open()) 
    {
        std::cerr << "\nerror: unable to open " << filename << " for reading!\n";
        exit(1);
    }

    std::streampos pos = inputFile.tellg();
    size_t inputSize = (size_t)pos;
    char * memBlock = new char [inputSize + 1];

    inputFile.seekg (0, std::ios::beg);
    inputFile.read (memBlock, inputSize);
    inputFile.close();
    memBlock[inputSize] = '\x0';

    // compile
    nvrtcProgram prog;
    NVRTC_SAFE_CALL("nvrtcCreateProgram", nvrtcCreateProgram(&prog, memBlock,
                                                     filename, 0, NULL, NULL));
    nvrtcResult res = nvrtcCompileProgram(prog, argc, argv);

    // dump log
    size_t logSize;
    NVRTC_SAFE_CALL("nvrtcGetProgramLogSize", nvrtcGetProgramLogSize(prog, &logSize));
    char *log = (char *) malloc(sizeof(char) * logSize + 1);
    NVRTC_SAFE_CALL("nvrtcGetProgramLog", nvrtcGetProgramLog(prog, log));
    log[logSize] = '\x0';

    /*
    std::cerr << "\n compilation log ---\n";
    std::cerr << log;
    std::cerr << "\n end log ---\n";
    */
    free(log);

    NVRTC_SAFE_CALL("nvrtcCompileProgram", res);
    // fetch PTX
    size_t ptxSize;
    NVRTC_SAFE_CALL("nvrtcGetPTXSize", nvrtcGetPTXSize(prog, &ptxSize));
    char *ptx = (char *) malloc(sizeof(char) * ptxSize);
    NVRTC_SAFE_CALL("nvrtcGetPTX", nvrtcGetPTX(prog, ptx));
    NVRTC_SAFE_CALL("nvrtcDestroyProgram", nvrtcDestroyProgram(&prog));
    *ptxResult = ptx;
    *ptxResultSize = ptxSize;
}

CUmodule loadPTX(char *ptx, int argc, char **argv)
{
    CUmodule module;
    CUcontext context;
    int major = 0, minor = 0;
    char deviceName[256];

    // Picks the best CUDA device available
    CUdevice cuDevice = findCudaDeviceDRV(argc, (const char **)argv);

    // get compute capabilities and the devicename
    checkCudaErrors(cuDeviceComputeCapability(&major, &minor, cuDevice));
    checkCudaErrors(cuDeviceGetName(deviceName, 256, cuDevice));
    printf("> GPU Device has SM %d.%d compute capability\n", major, minor);

    checkCudaErrors(cuInit(0));
    checkCudaErrors(cuDeviceGet(&cuDevice, 0));
    checkCudaErrors(cuCtxCreate(&context, 0, cuDevice));

    checkCudaErrors(cuModuleLoadDataEx(&module, ptx, 0, 0, 0));

    return module;
}

#endif

