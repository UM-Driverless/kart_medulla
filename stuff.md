# Build Issues and Solutions

## Ninja Build Tool Conflict (2025-11-02)

### Problem
PlatformIO builds were failing with CMake errors about incompatible ninja versions:
```
CMake Error at .../cmake-3.31/Modules/CMakeDetermineSystem.cmake:146 (message):
  Generator
    Ninja
  does not match the generator used previously:
    Ninja Multi-Config
```

### Root Cause
- Miniforge/conda installation at `/Users/rubenayla/miniforge3` was adding its own ninja to PATH
- This ninja was incompatible with the Homebrew ninja that PlatformIO expected
- CMake was caching the wrong ninja generator in build files
- Even after changing PATH in the shell, PlatformIO/CMake subprocesses were inheriting the old environment

### Solution Steps
1. **Remove miniforge from PATH**: Edited `~/.zshrc` to comment out miniforge initialization
2. **Clean build artifacts**: Deleted `.pio/build` directory to remove CMake cache
3. **Restart terminal completely**: Quit and reopen terminal app (not just new tab/window)
4. **Verify environment**:
   - `which ninja` → should be `/opt/homebrew/bin/ninja`
   - `env | grep LDFLAGS` → should be empty (no miniforge paths)
5. **Rebuild**: `~/.platformio/penv/bin/pio run` → SUCCESS

### Key Insight
The terminal environment must be completely restarted (quit the app) for all subprocess environments to be updated. Just opening a new tab or sourcing the profile is not sufficient.

### Prevention
- Keep conda/miniforge environments isolated
- Don't let conda modify the default PATH in shell profiles
- Use Homebrew-installed build tools for consistency with PlatformIO

### Build Stats After Fix
- RAM usage: 11.8% (38,816 bytes)
- Flash usage: 81.0% (1,061,738 bytes)
- Build time: 0.97 seconds

### Optional Convenience
Add PlatformIO to PATH in `~/.zshrc`:
```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
```
