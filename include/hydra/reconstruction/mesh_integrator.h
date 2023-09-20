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

#include "hydra/reconstruction/mesh_integrator_config.h"

namespace voxblox {
class ThreadSafeIndex;
}  // namespace voxblox

namespace hydra {

class VolumetricMap;

class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeshIntegrator(const MeshIntegratorConfig& config);

  virtual ~MeshIntegrator() = default;

  virtual void generateMesh(VolumetricMap& map,
                            bool only_mesh_updated_blocks,
                            bool clear_updated_flag) const;

  void allocateBlocks(const voxblox::BlockIndexList& blocks, VolumetricMap& map) const;

  void showUpdateInfo(const VolumetricMap& map,
                      const voxblox::BlockIndexList& blocks,
                      int verbosity) const;

  void launchThreads(const voxblox::BlockIndexList& blocks,
                     bool interior_pass,
                     VolumetricMap& map) const;

  void processInterior(const voxblox::BlockIndexList& blocks,
                       VolumetricMap* map,
                       voxblox::ThreadSafeIndex* index_getter) const;

  void processExterior(const voxblox::BlockIndexList& blocks,
                       VolumetricMap* map,
                       voxblox::ThreadSafeIndex* index_getter) const;

  virtual void meshBlockInterior(const voxblox::BlockIndex& block_index,
                                 const voxblox::VoxelIndex& voxel_index,
                                 VolumetricMap& map) const;

  virtual void meshBlockExterior(const voxblox::BlockIndex& block_index,
                                 const voxblox::VoxelIndex& voxel_index,
                                 VolumetricMap& map) const;

 protected:
  const MeshIntegratorConfig config_;
  Eigen::Matrix<int, 3, 8> cube_index_offsets_;
  mutable Eigen::Matrix<float, 3, 8> cube_coord_offsets_;
};

}  // namespace hydra
