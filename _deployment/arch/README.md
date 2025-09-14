# Gyroflow for Arch Linux

This directory contains build configurations and packaging files for Arch Linux.

## Building from Source

### Prerequisites

Install the required dependencies:

```bash
sudo pacman -S --needed base-devel git cargo just cmake ninja clang llvm python python-pip p7zip
sudo pacman -S --needed qt6-base qt6-declarative qt6-quickcontrols2 qt6-svg qt6-wayland
sudo pacman -S --needed ffmpeg libva libvdpau mesa ocl-icd libpulse alsa-lib
sudo pacman -S --needed libxkbcommon fontconfig freetype2 mesa-libgl glfw-x11
sudo pacman -S --needed libpng libjpeg-turbo libtiff libwebp libc++
sudo pacman -S --needed libgl libglvnd
```

### Build Instructions

**Note**: This build configuration uses vcpkg for OpenCV to maintain consistency with the original Linux configuration. vcpkg will build its own static libraries including PNG.

1. Clone the repository:
   ```bash
   git clone https://github.com/gyroflow/gyroflow.git
   cd gyroflow
   ```

2. Install dependencies and build:
   ```bash
   just arch install-deps
   just arch build
   ```

3. Run the application:
   ```bash
   just arch run
   ```

4. Clean build artifacts (if needed):
   ```bash
   just arch clean
   ```

5. Create a deployment package:
   ```bash
   just arch deploy
   ```

## Using the PKGBUILD

To build and install using makepkg:

```bash
cd _deployment/arch
makepkg -si
```

This will:
- Download the source code
- Install build dependencies
- Build the application
- Create a package
- Install the package

## AppImage

The build process also creates an AppImage that can be run on any Linux distribution:

```bash
just arch deploy
# The AppImage will be created in _deployment/_binaries/
```

## Docker Build

To build in a Docker container:

```bash
just arch deploy docker
```

## Architecture Support

- x86_64 (AMD64)
- aarch64 (ARM64)

## Troubleshooting

### Missing Dependencies

If you encounter missing library errors, ensure all dependencies are installed:

```bash
sudo pacman -S --needed $(pacman -Qq | grep -E '^(qt6-|ffmpeg|libva|libvdpau|mesa|ocl-icd|libpulse|alsa-lib|libxkbcommon|fontconfig|freetype2|mesa-libgl|glfw|libpng|libjpeg|libtiff|libwebp|libc\+\+)')
```

### OpenCL Issues

If you have issues with OpenCL, install the appropriate driver:

```bash
# For Intel GPUs
sudo pacman -S intel-compute-runtime

# For AMD GPUs
sudo pacman -S rocm-opencl-runtime

# For NVIDIA GPUs
sudo pacman -S opencl-nvidia
```

### Wayland Support

For Wayland support, ensure you have the necessary packages:

```bash
sudo pacman -S --needed qt6-wayland wayland-protocols
```

### Graphics/GPU Issues

If you encounter EGL or GPU initialization errors:

```bash
# Install graphics drivers and libraries
sudo pacman -S --needed libgl libglvnd mesa

# For Intel GPUs
sudo pacman -S --needed mesa-vulkan-intel

# For AMD GPUs  
sudo pacman -S --needed mesa-vulkan-radeon

# For NVIDIA GPUs
sudo pacman -S --needed nvidia-utils
```

### Running in Headless Environment

If running without a display (e.g., in Docker or SSH), you may need to set up a virtual display:

```bash
# Install Xvfb for virtual display
sudo pacman -S --needed xorg-server-xvfb

# Run with virtual display
xvfb-run -a just arch run
```
