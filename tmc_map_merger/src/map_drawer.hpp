/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TMC_MAP_MERGER_MAP_MERGER_MAP_DRAWER_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_MAP_DRAWER_HPP_

#include "map.hpp"

namespace { // NOLINT

template<typename Plotter>
void LineHigh(const Plotter& plotter, int8_t data, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
  int32_t dx = x1 - x0;
  int32_t dy = y1 - y0;
  int32_t xi = 1;
  if (dx < 0) {
    xi = -1;
    dx = -dx;
  }
  int32_t D = 2*dx - dy;
  int32_t x = x0;
  for (int32_t y = y0; y < y1; ++y) {
    plotter.Plot(data, x, y);
    if (D > 0) {
      x = x + xi;
      D = D - 2 * dy;
    }
    D = D + 2 * dx;
  }
}

template<typename Plotter>
void LineLow(const Plotter& plotter, int8_t data, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
  int32_t dx = x1 - x0;
  int32_t dy = y1 - y0;
  int32_t yi = 1;
  if (dy < 0) {
    yi = -1;
    dy = -dy;
  }
  int32_t D = 2*dy - dx;
  int32_t y = y0;
  for (int32_t x = x0; x < x1; ++x) {
    plotter.Plot(data, x, y);
    if (D > 0) {
      y = y + yi;
      D = D - 2 * dx;
    }
    D = D + 2 * dy;
  }
}

}  // anonymous namespace

namespace tmc_map_merger {

namespace drawer {


template<typename MapOp>
struct UnsafeMapPlotter {
  explicit UnsafeMapPlotter(const MapOp& o)
      : op(o) {}
  void Plot(int8_t data, int32_t x, int32_t y) const {
    op.Update(y * op.GetInfo().width + x, data);
  }
  const MapOp op;
};

template<typename MapOp>
struct MapPlotter {
  explicit MapPlotter(const MapOp& o)
      : op(o) {}
  void Plot(int8_t data, int32_t x, int32_t y) const {
    const MapInfo& info = op.GetInfo();
    if (x >= 0 &&
        x < info.width &&
        y >= 0 &&
        y < info.height) {
      op.Update(y * info.width + x, data);
    }
  }
  const MapOp op;
};

// Clip the line within the region using the Cohen-Sutherland algorithm
// see https://en.wikipedia.org/wiki/Cohen-Sutherland_algorithm
typedef int OutCode;
const OutCode kInside = 0;  // 0000
const OutCode kLeft = 1;    // 0001
const OutCode kRight = 2;   // 0010
const OutCode kBottom = 4;  // 0100
const OutCode kTop = 8;     // 1000

template<typename T>
OutCode ComputeOutCode(T x, T y, T width, T height) {
  OutCode code;
  code = kInside;
  if (x < 0.0) {
    code |= kLeft;
  } else if (x > width) {
    code |= kRight;
  }
  if (y < 0.0) {
    code |= kBottom;
  } else if (y > height) {
    code |= kTop;
  }
  return code;
}

template<typename T>
bool Clip(T& x0, T& y0, T& x1, T& y1, T width, T height) {
  OutCode outcode0 = ComputeOutCode(x0, y0, width, height);
  OutCode outcode1 = ComputeOutCode(x1, y1, width, height);
  bool is_inside = false;
  while (true) {
    if (!(outcode0 | outcode1)) {
      // If OR is 0, both endpoints are inside
      is_inside = true;
      break;
    } else if (outcode0 & outcode1) {
      // If AND is non-zero, both endpoints are in the outside region
      // (Both have the same region attribute of kLeft, kRight, kBottom, kTop)
      break;
    } else {
      T x;
      T y;
      // Select the one outside the region
      OutCode outcodeOut = outcode0 ? outcode0 : outcode1;
      // Find the intersection with the region boundary and update
      // Since it is guaranteed that the two endpoints are in different regions
      // No need to check for division by zero
      if (outcodeOut & kTop) {
        x = x0 + (x1 - x0) * static_cast<double>(height - y0) / (y1 - y0);
        y = height;
      } else if (outcodeOut & kBottom) {
        x = x0 + (x1 - x0) * static_cast<double>(0.0 - y0) / (y1 - y0);
        y = 0.0;
      } else if (outcodeOut & kRight) {
        y = y0 + (y1 - y0) * static_cast<double>(width - x0) / (x1 - x0);
        x = width;
      } else if (outcodeOut & kLeft) {
        y = y0 + (y1 - y0) * static_cast<double>(0.0 - x0) / (x1 - x0);
        x = 0.0;
      }
      // Update the point that was outside the region
      if (outcodeOut == outcode0) {
        x0 = x;
        y0 = y;
        outcode0 = ComputeOutCode(x0, y0, width, height);
      } else {
        x1 = x;
        y1 = y;
        outcode1 = ComputeOutCode(x1, y1, width, height);
      }
    }
  }
  return is_inside;
}

template<typename Plotter>
void Line(const Plotter& plotter, int8_t data, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
  if (std::abs(y1 - y0) < std::abs(x1 - x0)) {
    if (x0 > x1) {
      LineLow(plotter, data, x1, y1, x0, y0);
    } else {
      LineLow(plotter, data, x0, y0, x1, y1);
    }
  } else {
    if (y0 > y1) {
      LineHigh(plotter, data, x1, y1, x0, y0);
    } else {
      LineHigh(plotter, data, x0, y0, x1, y1);
    }
  }
}

template<typename Plotter>
void Circle(const Plotter& plotter, int8_t data, int32_t x0, int32_t y0, int32_t radius, bool fill) {
  int32_t x = radius;
  int32_t y = 0;
  int32_t F = -2 * radius + 3;
  int32_t i;

  while (x >= y) {
    if (fill) {
      for (i = -x; i <= x; ++i) {
        plotter.Plot(data, x0 + i, y0 - y);
        plotter.Plot(data, x0 + i, y0 + y);
      }
      for (i = -y; i <= y; ++i) {
        plotter.Plot(data, x0 + i, y0 - x);
        plotter.Plot(data, x0 + i, y0 + x);
      }
    } else {
      plotter.Plot(data, x0 - x, y0 - y);
      plotter.Plot(data, x0 + x, y0 - y);
      plotter.Plot(data, x0 - x, y0 + y);
      plotter.Plot(data, x0 + x, y0 + y);
      plotter.Plot(data, x0 - y, y0 - x);
      plotter.Plot(data, x0 + y, y0 - x);
      plotter.Plot(data, x0 - y, y0 + x);
      plotter.Plot(data, x0 + y, y0 + x);
    }
    if (F >= 0) {
      x--;
      F -= 4 * x;
    }
    y++;
    F += 4 * y + 2;
  }
}

}  // namespace drawer

}  // namespace tmc_map_merger

#endif
