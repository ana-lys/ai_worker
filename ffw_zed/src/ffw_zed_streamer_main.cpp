#include <dlfcn.h>
#include <iostream>

int main(int argc, char **argv) {
    void* handle = dlopen("libffw_zed_streamer_core.so", RTLD_NOW);
    if (!handle) {
        std::cerr << "Failed to load libffw_zed_streamer_core.so: " << dlerror() << "\n";
        return 1;
    }
    
    typedef int (*StartStreamFunc)(int, char**);
    StartStreamFunc start_stream = (StartStreamFunc)dlsym(handle, "start_stream");
    
    if (!start_stream) {
        std::cerr << "Failed to find start_stream: " << dlerror() << "\n";
        dlclose(handle);
        return 1;
    }
    
    int ret = start_stream(argc, argv);
    dlclose(handle);
    return ret;
}
