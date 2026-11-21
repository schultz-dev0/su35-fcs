// Minimal stand-in for <windows.h> so the mod's flight model source compiles with g++ on Linux.
// Our own code: it provides only what su35_xfm.cpp touches. The flight model itself is not changed.
#pragma once

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

typedef void *HMODULE;
typedef const char *LPCSTR;
typedef unsigned long DWORD;
typedef int BOOL;
typedef void *HINSTANCE;
typedef void *LPVOID;
#define WINAPI

#define MAX_PATH 260
#define TRUE 1
#define FALSE 0
#define GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS 0x4
#define GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT 0x2

// The mod exports its ed_fm_* functions with __declspec(dllexport). Here they must stay visible in the shared
// library so the S-function can look them up by name, as DCS does.
#define __declspec(x) __attribute__((visibility("default")))

namespace shim {

// Where the flight model finds SU35XFM.cfg (config_dir) and where its log goes (run_dir). Set by the harness
// before ed_fm_configure() is called.
inline std::string &config_dir()
{
    static std::string dir = ".";
    return dir;
}

inline std::string &run_dir()
{
    static std::string dir = ".";
    return dir;
}

inline std::string base_name(const std::string &path)
{
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

inline bool ends_with(const std::string &text, const char *tail)
{
    const size_t n = strlen(tail);
    return text.size() >= n && text.compare(text.size() - n, n, tail) == 0;
}

// The mod builds Windows paths with backslashes. Reads: turn them into slashes. Writes: the flight recorder
// (xfm-*.csv) is refused so the harness never writes into a DCS install; the log goes to run_dir.
inline FILE *open_file(const char *path, const char *mode)
{
    std::string fixed(path);
    for (char &c : fixed) {
        if (c == '\\') c = '/';
    }
    if (mode[0] != 'w') return fopen(fixed.c_str(), mode);

    const std::string name = base_name(fixed);
    if (name.compare(0, 4, "xfm-") == 0 && ends_with(name, ".csv")) return nullptr;
    return fopen((run_dir() + "/" + name).c_str(), mode);
}

}  // namespace shim

#define fopen(path, mode) shim::open_file(path, mode)

// No CockpitBase.dll in the harness: every lookup fails, so cockpit_read and cockpit_set_arg stay null and the
// flight model skips everything that needs the cockpit.
inline HMODULE GetModuleHandleA(LPCSTR) { return nullptr; }
inline void *GetProcAddress(HMODULE, LPCSTR) { return nullptr; }

inline BOOL GetModuleHandleExA(DWORD, LPCSTR, HMODULE *module)
{
    *module = (HMODULE)1;
    return TRUE;
}

// ed_fm_configure cuts this at the last backslash to find its folder, so give it a fake DLL name inside config_dir.
inline DWORD GetModuleFileNameA(HMODULE, char *buffer, DWORD size)
{
    const std::string fake = shim::config_dir() + "\\SU35FM.dll";
    snprintf(buffer, size, "%s", fake.c_str());
    return (DWORD)strlen(buffer);
}
