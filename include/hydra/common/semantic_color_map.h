/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <voxblox/core/common.h>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace voxblox {
std::ostream& operator<<(std::ostream& out, const Color& color);
}  // namespace voxblox

namespace hydra {

// For unordered map/set using Color as a key.
struct ColorHasher {
  size_t operator()(const voxblox::Color& k) const;
};

// For unordered_map/set using Color as a key
struct ColorEqual {
  bool operator()(const voxblox::Color& lhs, const voxblox::Color& rhs) const;
};

// based on code from kimera-semantics
class SemanticColorMap {
 public:
  using Ptr = std::unique_ptr<SemanticColorMap>;
  using ColorSet = std::unordered_set<voxblox::Color, ColorHasher, ColorEqual>;
  using LabelToColorMap = std::unordered_map<uint32_t, voxblox::Color>;
  using ColorToLabelMap =
      std::unordered_map<voxblox::Color, uint32_t, ColorHasher, ColorEqual>;

  SemanticColorMap();

  SemanticColorMap(const ColorToLabelMap& map,
                   const voxblox::Color& unknown_color = {});

  std::optional<uint32_t> getLabelFromColor(const voxblox::Color& color) const;

  voxblox::Color getColorFromLabel(const uint32_t& label) const;

  size_t getNumLabels() const;

  bool isValid() const;

  inline operator bool() const { return isValid(); }

  std::string toString() const;

 public:
  static SemanticColorMap::Ptr randomColors(size_t num_labels,
                                            const voxblox::Color& unknown = {});

  static SemanticColorMap::Ptr fromCsv(const std::string& filename,
                                       const voxblox::Color& unknown = {},
                                       bool skip_first = true,
                                       char delimiter = ',');

 private:
  uint32_t max_label_;
  ColorToLabelMap color_to_label_;
  LabelToColorMap label_to_color_;
  voxblox::Color unknown_color_;

  mutable ColorSet unknown_colors_;
  mutable std::unordered_set<uint32_t> unknown_labels_;
};

std::ostream& operator<<(std::ostream& out, const SemanticColorMap& cmap);

}  // namespace hydra