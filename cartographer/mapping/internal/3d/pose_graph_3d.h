/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
#include "cartographer/mapping/internal/work_queue.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping in 3D:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class PoseGraph3D : public PoseGraph {
 public:
  PoseGraph3D(
      const proto::PoseGraphOptions& options,
      std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph3D() override;

  PoseGraph3D(const PoseGraph3D&) = delete;
  PoseGraph3D& operator=(const PoseGraph3D&) = delete;

  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps)
      ;

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
      ;
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override
      ;
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override
      ;
  void AddLandmarkData(int trajectory_id,
                       const sensor::LandmarkData& landmark_data) override
      ;

  void DeleteTrajectory(int trajectory_id) override;
  void FinishTrajectory(int trajectory_id) override;
  bool IsTrajectoryFinished(int trajectory_id) const override
      ;
  void FreezeTrajectory(int trajectory_id) override;
  bool IsTrajectoryFrozen(int trajectory_id) const override
      ;
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const proto::Submap& submap) override;
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const proto::Node& node) override;
  void SetTrajectoryDataFromProto(const proto::TrajectoryData& data) override;
  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id) override;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override;
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() const override
      ;
  PoseGraph::SubmapData GetSubmapData(const SubmapId& submap_id) const
       override;
  MapById<SubmapId, SubmapData> GetAllSubmapData() const
       override;
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
       override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
       override;
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
      ;
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
      ;
  std::map<int, TrajectoryState> GetTrajectoryStates() const override
      ;
  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
      ;
  void SetLandmarkPose(const std::string& landmark_id,
                       const transform::Rigid3d& global_pose,
                       const bool frozen = false) override
      ;
  sensor::MapByTime<sensor::ImuData> GetImuData() const override
      ;
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
      ;
  sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
      const override ;
  std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  GetLandmarkNodes() const override ;
  std::map<int, TrajectoryData> GetTrajectoryData() const override;

  std::vector<Constraint> constraints() const override ;
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time) override
      ;
  void SetGlobalSlamOptimizationCallback(
      PoseGraphInterface::GlobalSlamOptimizationCallback callback) override;
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const
      ;

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 protected:
  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() 
      ;

 private:
  MapById<SubmapId, SubmapData> GetSubmapDataUnderLock() const
      ;

  // Handles a new work item.
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item)
       ;

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id)
      ;

  // Appends the new node and submap (if needed) to the internal data stuctures.
  NodeId AppendNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps,
      const transform::Rigid3d& optimized_pose) ;

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps)
      ;

  // Adds constraints for a node, and starts scan matching in the background.
  WorkItem::Result ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap3D>> insertion_submaps,
      bool newly_finished_submap) ;

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
      ;

  // Deletes trajectories waiting for deletion. Must not be called during
  // constraint search.
  void DeleteTrajectoriesIfNeeded() ;

  // Runs the optimization, executes the trimmers and processes the work queue.
  void HandleWorkQueue(const constraints::ConstraintBuilder3D::Result& result)
       ;

  // Process pending tasks in the work queue on the calling thread, until the
  // queue is either empty or an optimization is required.
  void DrainWorkQueue() 
      ;

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() ;

  bool CanAddWorkItemModifying(int trajectory_id)
      ;

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec3D>& global_submap_poses,
      int trajectory_id) const ;

  PoseGraph::SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
      ;

  common::Time GetLatestNodeTime(const NodeId& node_id,
                                 const SubmapId& submap_id) const
      ;

  // Logs histograms for the translational and rotational residual of node
  // poses.
  void LogResidualHistograms() const ;

  // Updates the trajectory connectivity structure with a new constraint.
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      ;

  const proto::PoseGraphOptions options_;
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  mutable absl::Mutex mutex_;
  absl::Mutex work_queue_mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<WorkQueue> work_queue_ ;

  // We globally localize a fraction of the nodes from each trajectory.
  absl::flat_hash_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ ;

  // Number of nodes added since last loop closure.
  int num_nodes_since_last_loop_closure_  = 0;

  // Current optimization problem.
  std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem_;
  constraints::ConstraintBuilder3D constraint_builder_;

  // Thread pool used for handling the work queue.
  common::ThreadPool* const thread_pool_;

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ ;

  PoseGraphData data_ ;

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(PoseGraph3D* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const override
        ;
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const override
        ;
    const std::vector<Constraint>& GetConstraints() const override
        ;
    void TrimSubmap(const SubmapId& submap_id)
         override;
    bool IsFinished(int trajectory_id) const override
        ;

    void SetTrajectoryState(int trajectory_id, TrajectoryState state) override
        ;

   private:
    PoseGraph3D* const parent_;
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_
