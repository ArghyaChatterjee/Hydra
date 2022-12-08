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
#include "hydra_topology/topology_server_visualizer.h"

namespace hydra {
namespace topology {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

TopologyServerVisualizer::TopologyServerVisualizer(const std::string& ns) : nh_(ns) {
  pubs_.reset(new MarkerGroupPub(nh_));

  config_ = config_parser::load_from_ros_nh<TopologyVisualizerConfig>(nh_);
  config_.graph.layer_z_step = 0;
  config_.graph.color_places_by_distance = true;

  setupConfigServers();
}

void TopologyServerVisualizer::visualize(const GraphExtractor& extractor,
                                         const SceneGraphLayer& graph,
                                         const Layer<GvdVoxel>& gvd,
                                         const Layer<TsdfVoxel>& tsdf) {
  visualizeGraph(graph);
  visualizeGvd(gvd);
  visualizeGvdEdges(extractor, gvd);
  if (config_.show_block_outlines) {
    visualizeBlocks(gvd, tsdf);
  }
}

void TopologyServerVisualizer::visualizeError(const Layer<GvdVoxel>& lhs,
                                              const Layer<GvdVoxel>& rhs,
                                              double threshold) {
  Marker msg = makeErrorMarker(config_.gvd, config_.colormap, lhs, rhs, threshold);
  msg.header.frame_id = config_.world_frame;
  msg.header.stamp = ros::Time::now();
  msg.ns = "error_marker";

  if (msg.points.size()) {
    pubs_->publish("error_viz", msg);
  } else {
    LOG(INFO) << "no voxels with error above threshold";
  }
}

void TopologyServerVisualizer::visualizeGraph(const SceneGraphLayer& graph) {
  if (graph.nodes().empty()) {
    LOG(INFO) << "visualizing empty graph!";
    return;
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config_.world_frame;

  MarkerArray markers;

  Marker node_marker;

  const std::string node_ns = config_.topology_marker_ns + "_nodes";
  if (config_.gvd.color_nearest_vertices) {
    node_marker = makeCentroidMarkers(
        header,
        config_.graph_layer,
        graph,
        config_.graph,
        node_ns,
        [](const SceneGraphNode& node) {
          if (node.attributes<PlaceNodeAttributes>().voxblox_mesh_connections.empty()) {
            return NodeColor(0, 0, 0);
          } else {
            return NodeColor(0, 255, 0);
          }
        });
  } else {
    node_marker = makeCentroidMarkers(
        header, config_.graph_layer, graph, config_.graph, node_ns, config_.colormap);
  }

  markers.markers.push_back(node_marker);

  if (!graph.edges().empty()) {
    Marker edge_marker = makeLayerEdgeMarkers(header,
                                              config_.graph_layer,
                                              graph,
                                              config_.graph,
                                              NodeColor::Zero(),
                                              config_.topology_marker_ns + "_edges");
    markers.markers.push_back(edge_marker);
  }

  publishGraphLabels(graph);
  pubs_->publish("graph_viz", markers);
}

void TopologyServerVisualizer::visualizeGvd(const Layer<GvdVoxel>& gvd) const {
  Marker esdf_msg = makeEsdfMarker(config_.gvd, config_.colormap, gvd);
  esdf_msg.header.frame_id = config_.world_frame;
  esdf_msg.header.stamp = ros::Time::now();
  esdf_msg.ns = "gvd_visualizer";

  if (esdf_msg.points.size()) {
    pubs_->publish("esdf_viz", esdf_msg);
  } else {
    LOG(INFO) << "visualizing empty ESDF slice";
  }

  Marker gvd_msg = makeGvdMarker(config_.gvd, config_.colormap, gvd);
  gvd_msg.header.frame_id = config_.world_frame;
  gvd_msg.header.stamp = ros::Time::now();
  gvd_msg.ns = "gvd_visualizer";

  if (gvd_msg.points.size()) {
    pubs_->publish("gvd_viz", gvd_msg);
  } else {
    LOG(INFO) << "visualizing empty GVD slice";
  }

  Marker surface_msg = makeSurfaceVoxelMarker(config_.gvd, config_.colormap, gvd);
  surface_msg.header.frame_id = config_.world_frame;
  surface_msg.header.stamp = ros::Time::now();
  surface_msg.ns = "gvd_visualizer";

  if (surface_msg.points.size()) {
    pubs_->publish("surface_viz", surface_msg);
  } else {
    LOG(INFO) << "visualizing empty surface slice";
  }
}

void TopologyServerVisualizer::visualizeBlocks(const Layer<GvdVoxel>& gvd,
                                               const Layer<TsdfVoxel>& tsdf) const {
  Marker msg;
  if (config_.use_gvd_block_outlines) {
    msg = makeBlocksMarker(gvd, config_.outline_scale);
  } else {
    msg = makeBlocksMarker(tsdf, config_.outline_scale);
  }

  msg.header.frame_id = config_.world_frame;
  msg.header.stamp = ros::Time::now();
  msg.ns = "topology_server_blocks";
  pubs_->publish("voxel_block_viz", msg);
}

void TopologyServerVisualizer::visualizeGvdEdges(const GraphExtractor& graph,
                                                 const Layer<GvdVoxel>& gvd) const {
  auto msg = makeGvdEdgeMarker(gvd, graph.getGvdEdgeInfo(), graph.getNodeRootMap());
  msg.header.frame_id = config_.world_frame;
  msg.header.stamp = ros::Time::now();
  pubs_->publish("gvd_edge_viz", msg);
}

void TopologyServerVisualizer::publishGraphLabels(const SceneGraphLayer& graph) {
  if (!config_.graph_layer.use_label) {
    return;
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config_.world_frame;
  const std::string label_ns = config_.topology_marker_ns + "_labels";

  MarkerArray labels;
  for (const auto& id_node_pair : graph.nodes()) {
    const SceneGraphNode& node = *id_node_pair.second;
    Marker label =
        makeTextMarker(header, config_.graph_layer, node, config_.graph, label_ns);
    labels.markers.push_back(label);
  }

  std::set<int> current_ids;
  for (const auto& label : labels.markers) {
    current_ids.insert(label.id);
  }

  std::set<int> ids_to_delete;
  for (auto previous : previous_labels_) {
    if (!current_ids.count(previous)) {
      ids_to_delete.insert(previous);
    }
  }
  previous_labels_ = current_ids;

  MarkerArray delete_markers;
  for (auto to_delete : ids_to_delete) {
    Marker delete_label;
    delete_label.action = Marker::DELETE;
    delete_label.id = to_delete;
    delete_label.ns = label_ns;
    delete_markers.markers.push_back(delete_label);
  }

  pubs_->publish("graph_label_viz", delete_markers);
  pubs_->publish("graph_label_viz", labels);
}

void TopologyServerVisualizer::graphConfigCb(LayerConfig& config, uint32_t) {
  config_.graph_layer = config;
}

void TopologyServerVisualizer::colormapCb(ColormapConfig& config, uint32_t) {
  config_.colormap = config;
}

void TopologyServerVisualizer::gvdConfigCb(GvdVisualizerConfig& config, uint32_t) {
  config_.gvd = config;
  config_.graph.places_colormap_min_distance = config.gvd_min_distance;
  config_.graph.places_colormap_max_distance = config.gvd_max_distance;
}

void TopologyServerVisualizer::setupConfigServers() {
  startRqtServer(
      "~/gvd_visualizer", gvd_config_server_, &TopologyServerVisualizer::gvdConfigCb);

  startRqtServer("~/graph_visualizer",
                 graph_config_server_,
                 &TopologyServerVisualizer::graphConfigCb);

  startRqtServer(
      "~/visualizer_colormap", colormap_server_, &TopologyServerVisualizer::colormapCb);
}

}  // namespace topology
}  // namespace hydra
