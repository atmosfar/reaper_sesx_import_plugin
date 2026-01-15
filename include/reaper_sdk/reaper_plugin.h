#ifndef _REAPER_PLUGIN_H_
#define _REAPER_PLUGIN_H_

#if defined(_WIN32)
  #include <windows.h>
  #include <stdint.h>
#else
  #include <stdint.h>
  #include <string.h>
  #ifndef HWND
    typedef void *HWND;
  #endif
  #ifndef HINSTANCE
    typedef void *HINSTANCE;
  #endif
#endif

#ifndef REAPER_PLUGIN_HINSTANCE
  #define REAPER_PLUGIN_HINSTANCE HINSTANCE
#endif

typedef void *(*get_func_ptr)(const char *func_name);
typedef int (*register_func)(const char *reg_id, void *structure);

typedef struct reaper_plugin_info_t
{
  int caller_version; 
  HWND hwnd_main;
  register_func Register;
  get_func_ptr GetFunc;
} reaper_plugin_info_t;

// Modern REAPER Plugin Version
#define REAPER_PLUGIN_VERSION 0x20E

#define REAPER_PLUGIN_ENTRYPOINT ReaperPluginEntry

#ifdef _WIN32
  #define REAPER_PLUGIN_DLL_EXPORT __declspec(dllexport)
#else
  #define REAPER_PLUGIN_DLL_EXPORT __attribute__ ((visibility ("default")))
#endif

// ProjectStateContext interface
class ProjectStateContext
{
public:
  virtual ~ProjectStateContext() {}
  virtual void AddLine(const char *fmt, ...) = 0;
  virtual int GetLine(char *buf, int buflen) = 0;
  virtual int64_t GetOutputSize() = 0;
  virtual int GetTempFlag() = 0;
  virtual void SetTempFlag(int flag) = 0;
};

// project_import_register_t structure
typedef struct 
{
  bool (*WantProjectFile)(const char *fn);
  const char *(*EnumFileExtensions)(int i, char **descptr);
  int (*LoadProject)(const char *fn, ProjectStateContext *genstate);
} project_import_register_t;

#endif // _REAPER_PLUGIN_H_
