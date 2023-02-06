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
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3native.h>

#include <X11/extensions/XShm.h>
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <sys/shm.h>
#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// Samples include their own definitions of stb_image. Use
// STB_IMAGE_WRITE_STATIC to avoid issues with multiple definitions in the
// nvpro_core static lib at the cost of having the code exist multiple times.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include <stb_image_write.h>

#include "linux_file_dialog.h"

union Pixel {
  uint32_t data;
  struct {
    uint8_t r, g, b, a;
  } channels;
};

// Object to allocate and hold a shared memory XImage and a shared memory
// segment This allows reading/writing an XImage in one IPC call Check
// XShmQueryExtension() before using
class XShmImage {
 public:
  XShmImage(Display* display, int width, int height) : m_display{display} {
    // Allocate a shared XImage
    int screen = XDefaultScreen(m_display);
    Visual* visual = XDefaultVisual(m_display, screen);
    int depth = DefaultDepth(m_display, screen);
    m_image = XShmCreateImage(m_display, visual, depth, ZPixmap, nullptr,
                              &m_shmSegmentInfo, width, height);
    if (!m_image) {
      LOGE("Error: XShmCreateImage() failed\n");
      return;
    }

    // Create the shared memory, used by XShmGetImage()
    int permissions = 0600;
    m_shmID = shmget(IPC_PRIVATE, height * m_image->bytes_per_line,
                     IPC_CREAT | permissions);
    if (m_shmID == -1) {
      LOGE("Error: shmget() failed\n");
      return;
    }

    // Map the shared memory segment into the address space of this process
    m_shmAddr = shmat(m_shmID, 0, 0);
    if (reinterpret_cast<intptr_t>(m_shmAddr) == -1) {
      LOGE("Error: shmat() failed\n");
      return;
    }

    // Use the allocated shared memory for the XImage
    m_shmSegmentInfo.shmid = m_shmID;
    m_shmSegmentInfo.shmaddr = reinterpret_cast<char*>(m_shmAddr);
    m_shmSegmentInfo.readOnly = false;
    m_image->data = m_shmSegmentInfo.shmaddr;

    // Get the X server to attach the shared memory segment on its side and sync
    if (!XShmAttach(m_display, &m_shmSegmentInfo))
      LOGE("Error: XShmAttach() failed\n");
    if (!XSync(m_display, false))
      LOGE("Error: XSync() failed\n");
    return;
  }
  ~XShmImage() {
    if (m_image) {
      if (!XShmDetach(m_display, &m_shmSegmentInfo))
        LOGE("Error: XShmDetach() failed\n");
      if (!XDestroyImage(m_image))
        LOGE("Error: XDestroyImage() failed\n");
    }
    if (reinterpret_cast<intptr_t>(m_shmAddr) != -1 && shmdt(m_shmAddr) == -1)
      LOGE("Error: shmdt() failed\n");
    if (m_shmID != -1 && shmctl(m_shmID, IPC_RMID, nullptr) == -1)
      LOGE("Error: shmctl(IPC_RMID) failed\n");
  }

  // Get the X server to copy the window contents into the shared memory
  bool read(Window window) {
    return XShmGetImage(m_display, window, m_image, 0, 0, AllPlanes);
  }

  // In lieu of exceptions, call this after constructing to see if the
  // constructor failed
  bool valid() const {
    return m_shmID != -1 && reinterpret_cast<intptr_t>(m_shmAddr) != -1;
  }

  // Returns the XImage object to be accessed after calling read()
  XImage* image() const { return m_image; }

 private:
  int m_shmID{-1};
  void* m_shmAddr{reinterpret_cast<void*>(-1)};
  XShmSegmentInfo m_shmSegmentInfo{};
  Display* m_display{};
  XImage* m_image{};
};

void NVPSystem::windowScreenshot(struct GLFWwindow* glfwin,
                                 const char* filename) {
  const int bytesPerPixel = sizeof(Pixel);
  int width{};
  int height{};
  std::unique_ptr<XShmImage> shmImage;
  std::vector<Pixel> imageData;
  XImage* fallbackXImage{};

  Display* display = glfwGetX11Display();
  Window window = glfwGetX11Window(glfwin);
  glfwGetWindowSize(glfwin, &width, &height);

  if (XShmQueryExtension(display)) {
    // Use the shared memory extension if it is supported to avoid expensive
    // XGetPixel calls Shared memory allows X11 to copy all the image data at
    // once
    shmImage = std::make_unique<XShmImage>(display, width, height);
    if (shmImage->valid() &&
        bytesPerPixel * 8 == shmImage->image()->bits_per_pixel) {
      if (shmImage->read(window)) {
        XImage* ximg = shmImage->image();
        imageData.reserve(width * height);
        for (int y = 0; y < ximg->height; ++y) {
          Pixel* ximgData =
              reinterpret_cast<Pixel*>(ximg->data + ximg->bytes_per_line * y);
          imageData.insert(imageData.end(), ximgData, ximgData + ximg->width);
        }

        // bgr to rgb
        for (Pixel& pixel : imageData) {
          std::swap(pixel.channels.r, pixel.channels.b);
        }
      } else {
        LOGE(
            "Error: Failed to get window contents for screenshot. Falling back "
            "to XGetPixel()\n");
      }
    } else {
      LOGE(
          "Error: Failed to create XShm Image for screenshot. Falling back to "
          "XGetPixel()\n");
    }
  }

  if (imageData.empty()) {
    fallbackXImage =
        XGetImage(display, window, 0, 0, width, height, AllPlanes, ZPixmap);
    if (!fallbackXImage) {
      LOGE("Error: XGetImage() failed to get window contents for screenshot\n");
      return;
    }
    if (bytesPerPixel * 8 != fallbackXImage->bits_per_pixel) {
      LOGE(
          "Error: XGetImage() returned an image with %i bits per pixel but "
          "only %i is supported\n",
          fallbackXImage->bits_per_pixel, bytesPerPixel * 8);
      return;
    }
    imageData.reserve(width * height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        Pixel pixel;
        pixel.data = static_cast<uint32_t>(XGetPixel(fallbackXImage, x, y));
        std::swap(pixel.channels.r, pixel.channels.b);  // bgr to rgb
        pixel.channels.a = 0xff;                        // set full alpha
        imageData.push_back(pixel);
      }
    }
  }

  if (!stbi_write_png(filename, width, height, 4, imageData.data(),
                      width * bytesPerPixel)) {
    LOGE("Error: Writing %s failed\n", filename);
  }

  if (fallbackXImage && !XDestroyImage(fallbackXImage))
    LOGE("Error: XDestroyImage() failed\n");
}

void NVPSystem::windowClear(struct GLFWwindow* glfwin,
                            uint32_t r,
                            uint32_t g,
                            uint32_t b) {
  Window hwnd = glfwGetX11Window(glfwin);
  assert(0 && "not yet implemented");
}

static void fixSingleFilter(std::string* pFilter);

static std::vector<std::string> toFilterArgs(const char* exts) {
  // Convert exts list to filter format recognized by portable-file-dialogs
  // Try to match implemented nvpsystem on Windows behavior:
  // Alternate between human-readable descriptions and filter strings.
  // | separates strings
  // ; separates filters e.g. .png|.gif
  // Case-insensitive e.g. .png = .PNG = .pNg

  // Split strings by |
  std::vector<std::string> filterArgs(1);
  for (const char* pC = exts; pC != nullptr && *pC != '\0'; ++pC) {
    char c = *pC;
    if (c == '|')
      filterArgs.emplace_back();
    else
      filterArgs.back().push_back(c);
  }

  // Default arguments
  if (filterArgs.size() < 2) {
    filterArgs = {"All files", "*"};
  }

  // Split filters by ; and fix those filters.
  for (size_t i = 1; i < filterArgs.size(); i += 2) {
    std::string& arg = filterArgs[i];
    std::string newArg;
    std::string singleFilter;
    for (char c : arg) {
      if (c == ';') {
        fixSingleFilter(&singleFilter);
        newArg += std::move(singleFilter);
        singleFilter.clear();
        newArg += ' ';  // portable-file-dialogs uses spaces to separate...
                        // win32 wants no spaces in filters at all so presumably
                        // this is fine.
      } else {
        singleFilter.push_back(c);
      }
    }
    fixSingleFilter(&singleFilter);
    newArg += std::move(singleFilter);
    arg = std::move(newArg);
  }

  return filterArgs;
}

static void fixSingleFilter(std::string* pFilter) {
  // Make case insensitive.
  std::string newFilter;
  for (char c : *pFilter) {
    if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
      // replace c with [cC] to make case-insensitive. TODO: Unicode support,
      // not sure how to implement for multibyte utf-8 characters.
      newFilter.push_back('[');
      newFilter.push_back(c);
      newFilter.push_back(char(c ^ 32));
      newFilter.push_back(']');
    } else {
      newFilter.push_back(c);
    }
  }
  *pFilter = std::move(newFilter);
}

std::string NVPSystem::windowOpenFileDialog(struct GLFWwindow* glfwin,
                                            const char* title,
                                            const char* exts) {
  // Not sure yet how to use this; maybe make as a child window somehow?
  [[maybe_unused]] Window hwnd = glfwGetX11Window(glfwin);

  std::vector<std::string> filterArgs = toFilterArgs(exts);
  std::vector<std::string> resultVector =
      open_file(title, ".", filterArgs).result();
  assert(resultVector.size() <= 1);
  return resultVector.empty() ? "" : std::move(resultVector[0]);
}

std::string NVPSystem::windowSaveFileDialog(struct GLFWwindow* glfwin,
                                            const char* title,
                                            const char* exts) {
  // Not sure yet how to use this; maybe make as a child window somehow?
  [[maybe_unused]] Window hwnd = glfwGetX11Window(glfwin);

  std::vector<std::string> filterArgs = toFilterArgs(exts);
  return save_file(title, ".", filterArgs).result();
}

void NVPSystem::sleep(double seconds) {
  ::sleep(seconds);
}

void NVPSystem::platformInit() {}

void NVPSystem::platformDeinit() {}

static bool s_exePathInit = false;

std::string NVPSystem::exePath() {
  static std::string s_exePath;

  if (!s_exePathInit) {
    char modulePath[PATH_MAX];
    ssize_t modulePathLength = readlink("/proc/self/exe", modulePath, PATH_MAX);

    s_exePath =
        std::string(modulePath, modulePathLength > 0 ? modulePathLength : 0);

    size_t last = s_exePath.rfind('/');
    if (last != std::string::npos) {
      s_exePath = s_exePath.substr(0, last) + std::string("/");
    }

    s_exePathInit = true;
  }

  return s_exePath;
}
