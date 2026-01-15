#ifndef _REAPER_PLUGIN_FUNCTIONS_H_
#define _REAPER_PLUGIN_FUNCTIONS_H_

#include "reaper_plugin.h"

// Minimal definition for Exploration track
extern void (*ShowConsoleMsg)(const char* msg);
extern int (*ShowMessageBox)(const char* msg, const char* title, int type);

// Helper macro to import functions
#define IMPAPI(x) if (!((*(void **)&(x)) = rec->GetFunc(#x))) return 0;

#endif // _REAPER_PLUGIN_FUNCTIONS_H_
