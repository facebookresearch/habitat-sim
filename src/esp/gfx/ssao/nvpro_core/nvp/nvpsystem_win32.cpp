/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------

#include "nvpsystem.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

#include <commdlg.h>
#include <windows.h>
#include <windowsx.h>

#include "resources.h"

#include <io.h>
#include <stdio.h>
#include <algorithm>
#include <string>
#include <vector>

// Executables (but not DLLs) exporting this symbol with this value will be
// automatically directed to the high-performance GPU on Nvidia Optimus systems
// with up-to-date drivers
//
extern "C" {
_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}

// from https://docs.microsoft.com/en-us/windows/desktop/gdi/capturing-an-image

static int CaptureAnImage(HWND hWnd, const char* filename) {
  HDC hdcWindow;
  HDC hdcMemDC = NULL;
  HBITMAP hbmScreen = NULL;
  BITMAP bmpScreen;

  // Retrieve the handle to a display device context for the client
  // area of the window.
  hdcWindow = GetDC(hWnd);

  // Create a compatible DC which is used in a BitBlt from the window DC
  hdcMemDC = CreateCompatibleDC(hdcWindow);

  if (!hdcMemDC) {
    LOGE("CreateCompatibleDC has failed\n");
    goto done;
  }

  // Get the client area for size calculation
  RECT rcClient;
  GetClientRect(hWnd, &rcClient);

  // Create a compatible bitmap from the Window DC
  hbmScreen = CreateCompatibleBitmap(hdcWindow, rcClient.right - rcClient.left,
                                     rcClient.bottom - rcClient.top);

  if (!hbmScreen) {
    LOGE("CreateCompatibleBitmap Failed\n");
    goto done;
  }

  // Select the compatible bitmap into the compatible memory DC.
  SelectObject(hdcMemDC, hbmScreen);

  // Bit block transfer into our compatible memory DC.
  if (!BitBlt(hdcMemDC, 0, 0, rcClient.right - rcClient.left,
              rcClient.bottom - rcClient.top, hdcWindow, 0, 0, SRCCOPY)) {
    LOGE("BitBlt has failed\n");
    goto done;
  }

  {
    // Get the BITMAP from the HBITMAP
    GetObject(hbmScreen, sizeof(BITMAP), &bmpScreen);

    BITMAPFILEHEADER bmfHeader{};
    BITMAPINFOHEADER bi;

    bi.biSize = sizeof(BITMAPINFOHEADER);
    bi.biWidth = bmpScreen.bmWidth;
    bi.biHeight = bmpScreen.bmHeight;
    bi.biPlanes = 1;
    bi.biBitCount = 32;
    bi.biCompression = BI_RGB;
    bi.biSizeImage = 0;
    bi.biXPelsPerMeter = 0;
    bi.biYPelsPerMeter = 0;
    bi.biClrUsed = 0;
    bi.biClrImportant = 0;

    DWORD dwBmpSize = ((bmpScreen.bmWidth * bi.biBitCount + 31) / 32) * 4 *
                      bmpScreen.bmHeight;

    // Starting with 32-bit Windows, GlobalAlloc and LocalAlloc are implemented
    // as wrapper functions that call HeapAlloc using a handle to the process's
    // default heap. Therefore, GlobalAlloc and LocalAlloc have greater overhead
    // than HeapAlloc.
    HANDLE hDIB = GlobalAlloc(GHND, dwBmpSize);
    if (hDIB == 0) {
      LOGE("GlobalAlloc in CaptureAnImage has failed\n");
      goto done;
    }
    char* lpbitmap = (char*)GlobalLock(hDIB);

    // Gets the "bits" from the bitmap and copies them into a buffer
    // which is pointed to by lpbitmap.
    GetDIBits(hdcWindow, hbmScreen, 0, (UINT)bmpScreen.bmHeight, lpbitmap,
              (BITMAPINFO*)&bi, DIB_RGB_COLORS);

    // A file is created, this is where we will save the screen capture.
    HANDLE hFile = CreateFileA(filename, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
                               FILE_ATTRIBUTE_NORMAL, NULL);

    // Add the size of the headers to the size of the bitmap to get the total
    // file size
    DWORD dwSizeofDIB =
        dwBmpSize + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

    // Offset to where the actual bitmap bits start.
    bmfHeader.bfOffBits =
        (DWORD)sizeof(BITMAPFILEHEADER) + (DWORD)sizeof(BITMAPINFOHEADER);

    // Size of the file
    bmfHeader.bfSize = dwSizeofDIB;

    // bfType must always be BM for Bitmaps
    bmfHeader.bfType = 0x4D42;  // BM

    DWORD dwBytesWritten = 0;
    WriteFile(hFile, (LPSTR)&bmfHeader, sizeof(BITMAPFILEHEADER),
              &dwBytesWritten, NULL);
    WriteFile(hFile, (LPSTR)&bi, sizeof(BITMAPINFOHEADER), &dwBytesWritten,
              NULL);
    WriteFile(hFile, (LPSTR)lpbitmap, dwBmpSize, &dwBytesWritten, NULL);

    // Unlock and Free the DIB from the heap
    GlobalUnlock(hDIB);
    GlobalFree(hDIB);

    // Close the handle for the file that was created
    CloseHandle(hFile);
  }

  // Clean up
done:
  if (hbmScreen != 0) {
    DeleteObject(hbmScreen);
  }
  if (hdcMemDC != 0) {
    DeleteObject(hdcMemDC);
  }
  ReleaseDC(hWnd, hdcWindow);

  return 0;
}

void NVPSystem::windowScreenshot(struct GLFWwindow* glfwin,
                                 const char* filename) {
  if (!glfwin) {
    assert(!"Attempted to fall windowScreenshot() on null window!");
    return;
  }
  CaptureAnImage(glfwGetWin32Window(glfwin), filename);
}

void NVPSystem::windowClear(struct GLFWwindow* glfwin,
                            uint32_t r,
                            uint32_t g,
                            uint32_t b) {
  if (!glfwin) {
    assert(!"Attempted to fall windowClear() on null window!");
    return;
  }
  HWND hwnd = glfwGetWin32Window(glfwin);

  HDC hdcWindow = GetDC(hwnd);

  RECT rcClient;
  GetClientRect(hwnd, &rcClient);
  HBRUSH hbr = CreateSolidBrush(RGB(r, g, b));

  FillRect(hdcWindow, &rcClient, hbr);

  ReleaseDC(hwnd, hdcWindow);
  DeleteBrush(hbr);
}

static std::string fileDialog(struct GLFWwindow* glfwin,
                              const char* title,
                              const char* exts,
                              bool openToLoad) {
  if (!glfwin) {
    assert(!"Attempted to fall fileDialog() on null window!");
    return std::string();
  }
  HWND hwnd = glfwGetWin32Window(glfwin);

  std::vector<char> extsfixed;
  for (size_t i = 0; i < strlen(exts); i++) {
    if (exts[i] == '|') {
      extsfixed.push_back(0);
    } else {
      extsfixed.push_back(exts[i]);
    }
  }
  extsfixed.push_back(0);
  extsfixed.push_back(0);

  OPENFILENAME ofn;   // common dialog box structure
  char szFile[1024];  // buffer for file name

  // Initialize OPENFILENAME
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = hwnd;
  ofn.lpstrFile = szFile;
  // Set lpstrFile[0] to '\0' so that GetOpenFileName does not
  // use the contents of szFile to initialize itself.
  ofn.lpstrFile[0] = '\0';
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = extsfixed.data();
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = NULL;
  ofn.Flags = OFN_PATHMUSTEXIST;
  ofn.lpstrTitle = title;

  // Display the Open dialog box.

  if (openToLoad) {
    ofn.Flags |= OFN_FILEMUSTEXIST;
    if (GetOpenFileNameA(&ofn) == TRUE) {
      return ofn.lpstrFile;
    }
  } else {
    ofn.Flags |= OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn) == TRUE) {
      return ofn.lpstrFile;
    }
  }

  return std::string();
}

std::string NVPSystem::windowOpenFileDialog(struct GLFWwindow* glfwin,
                                            const char* title,
                                            const char* exts) {
  return fileDialog(glfwin, title, exts, true);
}

std::string NVPSystem::windowSaveFileDialog(struct GLFWwindow* glfwin,
                                            const char* title,
                                            const char* exts) {
  return fileDialog(glfwin, title, exts, false);
}

void NVPSystem::sleep(double seconds) {
  ::Sleep(DWORD(seconds * 1000.0));
}

void NVPSystem::platformInit() {
#ifdef MEMORY_LEAKS_CHECK
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_WNDW);
#endif
}

void NVPSystem::platformDeinit() {
#ifdef MEMORY_LEAKS_CHECK
  _CrtDumpMemoryLeaks();
#endif
}

static std::string s_exePath;
static bool s_exePathInit = false;

std::string NVPSystem::exePath() {
  if (!s_exePathInit) {
    char modulePath[MAX_PATH];
    size_t modulePathLength = GetModuleFileNameA(NULL, modulePath, MAX_PATH);
    s_exePath = std::string(modulePath, modulePathLength);

    std::replace(s_exePath.begin(), s_exePath.end(), '\\', '/');
    size_t last = s_exePath.rfind('/');
    if (last != std::string::npos) {
      s_exePath = s_exePath.substr(0, last) + std::string("/");
    }

    s_exePathInit = true;
  }

  return s_exePath;
}
