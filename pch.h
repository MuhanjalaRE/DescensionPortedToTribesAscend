// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

// add headers that you want to pre-compile here

#include "DX.h"
#include "ue.h"

#include "imgui.h"
#include "imgui_impl_dx9.h"
#include "imgui_impl_win32.h"

#include "Other/Keys/Keys.h"

#include <fstream>
#include <chrono>
#include <iostream>
#include <utility>

//#define USE_SOL
#ifdef USE_SOL
#include <sol/sol.hpp>
#endif

#include "SDK.h"

#include "detours.h"

#endif //PCH_H
