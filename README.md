# AI WORKER BSP

## Docker
- Install Docker Engine by following the official guide: https://docs.docker.com/engine/install/
- After installation, complete the Linux post-install steps: https://docs.docker.com/engine/install/linux-postinstall/

## Install JetPack BSP
### Set up the base environment
- When you run `docker_build.sh`, a folder named with the JetPack version is created in the parent directory, the NVIDIA JetPack BSP is downloaded there, and the patch files are applied.
  ```
  cd ai_worker/6.2.0
  ./docker_build.sh
  ```

- After the base environment setup is complete, move to the generated BSP folder to build the kernel and perform the flash.
  - Build the kernel
    ```
    cd ../../6.2.0
    cd Linux_for_Tegra
    ./docker_kernel_build.sh
    ```
  - Flash
    ```
    ./docker_flash_orin.sh -v v02
    ```