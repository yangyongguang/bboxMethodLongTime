#include "imm_ukf_pda.h"

ImmUkfPda::ImmUkfPda()
  : target_id_(0)
  ,  // assign unique ukf_id_ to each tracking targets
  init_(false),
  frame_count_(0),
  has_subscribed_vectormap_(false)
{
//   private_nh_.param<std::string>("tracking_frame", tracking_frame_, "world");
//   private_nh_.param<int>("life_time_threshold", life_time_threshold_, 8);
//   private_nh_.param<double>("gating_threshold", gating_threshold_, 9.22);
//   private_nh_.param<double>("gate_probability", gate_probability_, 0.99);
//   private_nh_.param<double>("detection_probability", detection_probability_, 0.9);
//   private_nh_.param<double>("static_velocity_threshold", static_velocity_threshold_, 0.5);
//   private_nh_.param<int>("static_num_history_threshold", static_num_history_threshold_, 3);
//   private_nh_.param<double>("prevent_explosion_threshold", prevent_explosion_threshold_, 1000);
//   private_nh_.param<double>("merge_distance_threshold", merge_distance_threshold_, 0.5);
//   private_nh_.param<bool>("use_sukf", use_sukf_, false);

//   // for vectormap assisted tracking
//   private_nh_.param<bool>("use_vectormap", use_vectormap_, false);
//   private_nh_.param<double>("lane_direction_chi_threshold", lane_direction_chi_threshold_, 2.71);
//   private_nh_.param<double>("nearest_lane_distance_threshold", nearest_lane_distance_threshold_, 1.0);
//   private_nh_.param<std::string>("vectormap_frame", vectormap_frame_, "map");

//   // rosparam for benchmark
//   private_nh_.param<bool>("is_benchmark", is_benchmark_, false);
//   private_nh_.param<std::string>("kitti_data_dir", kitti_data_dir_, "");
  if (is_benchmark_)
  {
    result_file_path_ = kitti_data_dir_ + "benchmark_results.txt";
    std::remove(result_file_path_.c_str());
  }
}

// 主函数
void ImmUkfPda::run()
{
//   pub_object_array_ = node_handle_.advertise<std::vector<BBox>>("/detection/objects", 1);
//   sub_detected_array_ = node_handle_.subscribe("/detection/fusion_tools/objects", 1, &ImmUkfPda::callback, this);

//   if (use_vectormap_)
//   {
//     vmap_.subscribe(private_nh_, vector_map::Category::point |
//                                  vector_map::Category::NODE  |
//                                  vector_map::Category::LANE, 1);
//   }
}

void ImmUkfPda::callback(const std::vector<BBox>& input)
{


//   bool success = updateNecessaryTransform();
//   if (!success)
//   {
//     fprintf(stderr, "Could not find coordiante transformation\n");
//     return;
//   }

  std::vector<BBox> transformed_input;
  std::vector<BBox> detected_objects_output;
  transformPoseToGlobal(input, transformed_input);
  // 全局 bbox 输入跟踪
  tracker(transformed_input, detected_objects_output);
  // 还原
  transformPoseToLocal(detected_objects_output);

  if (is_benchmark_)
  {
    dumpResultText(detected_objects_output);
  }
}

// bool ImmUkfPda::updateNecessaryTransform()
// {
//   bool success = true;
//   try
//   {
//     tf_listener_.waitForTransform(input_header_.frame_id, tracking_frame_, ros::Time(0), ros::Duration(1.0));
//     // 從 input_header 投射到 tracking_frame_ 坐標系的轉換
//     tf_listener_.lookupTransform(tracking_frame_, input_header_.frame_id, ros::Time(0), local2global_);
//   }
//   catch (tf::TransformException ex)
//   {
//     ROS_ERROR("%s", ex.what());
//     success = false;
//   }
//   if (use_vectormap_ && has_subscribed_vectormap_)
//   {
//     try
//     {
//       tf_listener_.waitForTransform(vectormap_frame_, tracking_frame_, ros::Time(0), ros::Duration(1.0));
//       tf_listener_.lookupTransform(vectormap_frame_, tracking_frame_, ros::Time(0), tracking_frame2lane_frame_);
//       tf_listener_.lookupTransform(tracking_frame_, vectormap_frame_, ros::Time(0), lane_frame2tracking_frame_);
//     }
//     catch (tf::TransformException ex)
//     {
//       ROS_ERROR("%s", ex.what());
//     }
//   }
//   return success;
// }

void ImmUkfPda::transformPoseToGlobal(const std::vector<BBox>& input,
                                      std::vector<BBox>& transformed_input)
{
  for (auto const &object: input)
  {
    Pose out_pose = getTransformedPose(object.pose);

    BBox dd;
    dd = object;
    dd.pose = out_pose;

    transformed_input.push_back(dd);
  }
}

void ImmUkfPda::transformPoseToLocal(std::vector<BBox>& detected_objects_output)
{

  for (auto& object : detected_objects_output)
  {
    Pose out_pose = getTransformedPose(object.pose);
    object.pose = out_pose;
  }
}

Pose ImmUkfPda::getTransformedPose(const Pose& in_pose)
{

}

void ImmUkfPda::measurementValidation(const std::vector<BBox>& input, UKF& target,
                                      const bool second_init, const Eigen::VectorXd& max_det_z,
                                      const Eigen::MatrixXd& max_det_s,
                                      std::vector<BBox>& object_vec,
                                      std::vector<bool>& matching_vec)
{
  // alert: different from original imm-pda filter, here picking up most likely measurement
  // if making it allows to have more than one measurement, you will see non semipositive definite covariance
  bool exists_smallest_nis_object = false;
  double smallest_nis = std::numeric_limits<double>::max();
  int smallest_nis_ind = 0;
  for (size_t i = 0; i < input.size(); i++)
  {
    double x = input[i].pose.position.x;
    double y = input[i].pose.position.y;

    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas << x, y;

    // 使用到了测量值
    Eigen::VectorXd diff = meas - max_det_z;
    double nis = diff.transpose() * max_det_s.inverse() * diff;
    // nis 值越小就说明但前检测的值距离预测的值越近
    // gating_threshold_ 为距离阈值 9.22， 论文中的
    if (nis < gating_threshold_)
    {
      if (nis < smallest_nis)
      {
        smallest_nis = nis;
        // 为预测的对象找最近距离的 测量对象
        target.object_ = input[i];
        smallest_nis_ind = i;
        // 找到了最小的距离， 在门限内的
        exists_smallest_nis_object = true;
      }
    }
  }
  if (exists_smallest_nis_object)
  {
    // 将已经被选择的测量对象标记为已经被匹配了
    matching_vec[smallest_nis_ind] = true;
    if (use_vectormap_ && has_subscribed_vectormap_)
    {
      BBox direction_updated_object;
      bool use_direction_meas =
          updateDirection(smallest_nis, target.object_, direction_updated_object, target);
      if (use_direction_meas)
      {
        object_vec.push_back(direction_updated_object);
      }
      else
      {
        object_vec.push_back(target.object_);
      }
    }
    else
    {
      object_vec.push_back(target.object_);
    }
  }
}

bool ImmUkfPda::updateDirection(const double smallest_nis, const BBox& in_object,
                                    BBox& out_object, UKF& target)
{
  bool use_lane_direction = false;
  target.is_direction_cv_available_ = false;
  target.is_direction_ctrv_available_ = false;
  target.checkLaneDirectionAvailability(out_object, lane_direction_chi_threshold_, use_sukf_);
  if (target.is_direction_cv_available_ || target.is_direction_ctrv_available_)
  {
    use_lane_direction = true;
  }
  return use_lane_direction;
}

void ImmUkfPda::updateTargetWithAssociatedObject(const std::vector<BBox>& object_vec,
                                                 UKF& target)
{
  target.lifetime_++;
  if (!target.object_.label.empty() && target.object_.label !="unknown")
  {
    target.label_ = target.object_.label;
  }
  updateTrackingNum(object_vec, target);
  // Occlusion 代表短的失去一帧目标
  if (target.tracking_num_ == TrackingState::Stable || target.tracking_num_ == TrackingState::Occlusion)
  {
    target.is_stable_ = true;
  }
}

void ImmUkfPda::updateBehaviorState(const UKF& target, const bool use_sukf, BBox& object)
{
  if(use_sukf)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CV;
  }
  else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else
  {
    object.behavior_state = MotionModel::RM;
  }
}

// 初始化 Tracker 将对象的中心坐标和 target_id_ 作为初始化的 ukf 跟踪对象的初始化条件
void ImmUkfPda::initTracker(const std::vector<BBox>& input, double timestamp)
{
  // 使用到了测量值
  for (size_t i = 0; i < input.size(); i++)
  {
    double px = input[i].pose.position.x;
    double py = input[i].pose.position.y;
    Eigen::VectorXd init_meas = Eigen::VectorXd(2);
    init_meas << px, py;

    UKF ukf;
    ukf.initialize(init_meas, timestamp, target_id_);
    targets_.push_back(ukf);
    target_id_++;
  }
  timestamp_ = timestamp;
  init_ = true;
}

void ImmUkfPda::secondInit(UKF& target, const std::vector<BBox>& object_vec, double dt)
{
  if (object_vec.size() == 0)
  {
    target.tracking_num_ = TrackingState::Die;
    return;
  }
  // record init measurement for env classification
  target.init_meas_ << target.x_merge_(0), target.x_merge_(1);

  // state update
  double target_x = object_vec[0].pose.position.x;
  double target_y = object_vec[0].pose.position.y;
  // x_merger 因该是上以时刻预测与测量合并后的状态值
  double target_diff_x = target_x - target.x_merge_(0);
  double target_diff_y = target_y - target.x_merge_(1);
  double target_yaw = atan2(target_diff_y, target_diff_x);
  double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
  double target_v = dist / dt;

  while (target_yaw > M_PI)
    target_yaw -= 2. * M_PI;
  while (target_yaw < -M_PI)
    target_yaw += 2. * M_PI;

  // 利用当前的测量， 跟新
  // 朝向角度直接是利用俩帧之间的中心点的矢量方向， 可以选择一个值， 来调整测量与预测的角度， 最好利用 tracking_point 
  // 这个是第二次初始化， 可能与后面的不太一样
  target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
  target.x_merge_(1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
  target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
  target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;

  target.tracking_num_++;
  return;
}

void ImmUkfPda::updateTrackingNum(const std::vector<BBox>& object_vec, UKF& target)
{
  // tracking_num_ 不会无限制增长的
  if (object_vec.size() > 0)
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      // 有关联到测量对象， 则跟踪帧加一
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }
  else
  {
    // 失去检测的目标， 对应帧策略
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Die;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      // 稳定的对象， 允许短暂的在 Stable 与 Die 之间的帧数， 失去目标， 
      // 如果增加到 Lost 的数目依然没有关联到目标，代表目标死亡了
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }

  return;
}

bool ImmUkfPda::probabilisticDataAssociation(const std::vector<BBox>& input, const double dt,
                                             std::vector<bool>& matching_vec,
                                             std::vector<BBox>& object_vec, UKF& target)
{
  double det_s = 0;
  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;
  bool success = true;

  if (use_sukf_)
  {
    max_det_z = target.z_pred_ctrv_;
    max_det_s = target.s_ctrv_;
    det_s = max_det_s.determinant();
  }
  else
  {
    // find maxDetS associated with predZ
    target.findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();
  }

  // prevent ukf not to explode
  if (std::isnan(det_s) || det_s > prevent_explosion_threshold_)
  {
    target.tracking_num_ = TrackingState::Die;
    success = false;
    return success;
  }

  bool is_second_init;
  //  对于新生的 tracking 需要二次
  if (target.tracking_num_ == TrackingState::Init)
  {
    is_second_init = true;
  }
  else
  {
    is_second_init = false;
  }

  // measurement gating 为每一个 预测预先找到 gate 范围内的测量集合
  measurementValidation(input, target, is_second_init, max_det_z, max_det_s, object_vec, matching_vec);

  // second detection for a target: update v and yaw
  if (is_second_init)
  {
    secondInit(target, object_vec, dt);
    success = false;
    return success;
  }

  updateTargetWithAssociatedObject(object_vec, target);

  if (target.tracking_num_ == TrackingState::Die)
  {
    success = false;
    return success;
  }
  return success;
}

void ImmUkfPda::makeNewTargets(const double timestamp, const std::vector<BBox>& input,
                               const std::vector<bool>& matching_vec)
{
  // 使用到了测量值
  for (size_t i = 0; i < input.size(); i++)
  {
    if (matching_vec[i] == false)
    {
      double px = input[i].pose.position.x;
      double py = input[i].pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_);
      ukf.object_ = input[i];
      targets_.push_back(ukf);
      target_id_++;
    }
  }
}

void ImmUkfPda::staticClassification()
{
  // 所有对象， 默认动态， 只要是被判定为静态， 则不能在被判断为动态
  // 如前方停靠的汽车， 如果正在行驶轨迹上， 则跟踪， 如果不再行驶轨迹上， 则不需要进行动静判别
  // 这样应该可以消除跟踪带来的花坛问题
  for (size_t i = 0; i < targets_.size(); i++)
  {
    // targets_[i].x_merge_(2) is referred for estimated velocity
    double current_velocity = std::abs(targets_[i].x_merge_(2));
    targets_[i].vel_history_.push_back(current_velocity);
    if (targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_threshold_)
    {
      int index = 0;
      double sum_vel = 0;
      double avg_vel = 0;
      // 取最后存储的速度， 取平均后使用
      for (auto rit = targets_[i].vel_history_.rbegin(); index < static_num_history_threshold_; ++rit)
      {
        index++;
        sum_vel += *rit;
      }
      avg_vel = double(sum_vel / static_num_history_threshold_);

      if(avg_vel < static_velocity_threshold_ && current_velocity < static_velocity_threshold_)
      {
        targets_[i].is_static_ = true;
      }
    }
  }
}

bool
ImmUkfPda::arePointsClose(const point& in_point_a,
                                const point& in_point_b,
                                float in_radius)
{
  return (fabs(in_point_a.x() - in_point_b.x()) <= in_radius) && (fabs(in_point_a.y() - in_point_b.y()) <= in_radius);
}

bool
ImmUkfPda::arePointsEqual(const point& in_point_a,
                               const point& in_point_b)
{
  return arePointsClose(in_point_a, in_point_b, CENTROID_DISTANCE);
}

bool
ImmUkfPda::isPointInPool(const std::vector<point>& in_pool,
                          const point& in_point)
{
  for(size_t j=0; j<in_pool.size(); j++)
  {
    if (arePointsEqual(in_pool[j], in_point))
    {
      return true;
    }
  }
  return false;
}

std::vector<BBox>
ImmUkfPda::removeRedundantObjects(const std::vector<BBox>& in_detected_objects,
                            const std::vector<size_t> in_tracker_indices)
{
  if (in_detected_objects.size() != in_tracker_indices.size())
    return in_detected_objects;

  std::vector<BBox> resulting_objects;

  std::vector<point> centroids;
  //create unique points
  for(size_t i=0; i<in_detected_objects.size(); i++)
  {
    point pt(in_detected_objects[i].pose.position.x, 
             in_detected_objects[i].pose.position.y, 
             in_detected_objects[i].pose.position.z);
    if(!isPointInPool(centroids, pt))
    {
      point pt(in_detected_objects[i].pose.position.x, 
          in_detected_objects[i].pose.position.y, 
          in_detected_objects[i].pose.position.z);
      centroids.push_back(pt);
    }
  }
  //assign objects to the points
  std::vector<std::vector<size_t>> matching_objects(centroids.size());
  for(size_t k=0; k<in_detected_objects.size(); k++)
  {
    const auto& object=in_detected_objects[k];
    for(size_t i=0; i< centroids.size(); i++)
    {
      point pt(object.pose.position.x, object.pose.position.y, object.pose.position.z);
      if (arePointsClose(pt, centroids[i], merge_distance_threshold_))
      {
        matching_objects[i].push_back(k);//store index of matched object to this point
      }
    }
  }
  //get oldest object on each point
  for(size_t i=0; i< matching_objects.size(); i++)
  {
    size_t oldest_object_index = 0;
    int oldest_lifespan = -1;
    std::string best_label;
    for(size_t j=0; j<matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
      if (current_lifespan > oldest_lifespan)
      {
        oldest_lifespan = current_lifespan;
        oldest_object_index = current_index;
      }
      if (!targets_[in_tracker_indices[current_index]].label_.empty() &&
        targets_[in_tracker_indices[current_index]].label_ != "unknown")
      {
        best_label = targets_[in_tracker_indices[current_index]].label_;
      }
    }
    // delete nearby targets except for the oldest target
    for(size_t j=0; j<matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      if(current_index != oldest_object_index)
      {
        targets_[in_tracker_indices[current_index]].tracking_num_= TrackingState::Die;
      }
    }
    BBox best_object;
    best_object = in_detected_objects[oldest_object_index];
    if (best_label != "unknown"
        && !best_label.empty())
    {
      best_object.label = best_label;
    }

    resulting_objects.push_back(best_object);
  }

  return resulting_objects;

}

void ImmUkfPda::makeOutput(const std::vector<BBox>& input,
                           const std::vector<bool> &matching_vec,
                           std::vector<BBox>& detected_objects_output)
{
  std::vector<BBox> tmp_objects;
  std::vector<size_t> used_targets_indices;
  for (size_t i = 0; i < targets_.size(); i++)
  {

    double tx = targets_[i].x_merge_(0);
    double ty = targets_[i].x_merge_(1);

    double tv = targets_[i].x_merge_(2);
    double tyaw = targets_[i].x_merge_(3);
    double tyaw_rate = targets_[i].x_merge_(4);

    while (tyaw > M_PI)
      tyaw -= 2. * M_PI;
    while (tyaw < -M_PI)
      tyaw += 2. * M_PI;

    // tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

    BBox dd;
    dd = targets_[i].object_;
    dd.id = targets_[i].ukf_id_;
    dd.velocity.linear.x = tv;
    dd.acceleration.linear.y = tyaw_rate;
    dd.velocity_reliable = targets_[i].is_stable_;
    dd.pose_reliable = targets_[i].is_stable_;


    // 目标稳定与否
    // 目标是否是动态的目标， 静态目标不跟踪
    if (!targets_[i].is_static_ && targets_[i].is_stable_)
    {
      // Aligh the longest side of dimentions with the estimated orientation
      if(targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y)
      {
        dd.dimensions.x = targets_[i].object_.dimensions.y;
        dd.dimensions.y = targets_[i].object_.dimensions.x;
      }

      dd.pose.position.x = tx;
      dd.pose.position.y = ty;

    //   if (!std::isnan(q[0]))
    //     dd.pose.orientation.x = q[0];
    //   if (!std::isnan(q[1]))
    //     dd.pose.orientation.y = q[1];
    //   if (!std::isnan(q[2]))
    //     dd.pose.orientation.z = q[2];
    //   if (!std::isnan(q[3]))
    //     dd.pose.orientation.w = q[3];
    dd.pose.yaw = tyaw;
    }
    // 根据概率， 判断使用了哪一个模型
    updateBehaviorState(targets_[i], use_sukf_, dd);

    // 保留稳定的跟踪对象， 和处于初始化与稳定之间的跟踪对象
    if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init &&
                                   targets_[i].tracking_num_ < TrackingState::Stable))
    {
      tmp_objects.push_back(dd);
      used_targets_indices.push_back(i);
    }
  }
  // 去除多余的跟踪对象
  detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
}

void ImmUkfPda::removeUnnecessaryTarget()
{
  std::vector<UKF> temp_targets;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ != TrackingState::Die)
    {
      temp_targets.push_back(targets_[i]);
    }
  }
  // 清楚 targets_ 的所有元素， 并将其空间缩短直至最小， 如果不考虑使用 list
  std::vector<UKF>().swap(targets_);
  targets_ = temp_targets;
}

void ImmUkfPda::dumpResultText(std::vector<BBox>& detected_objects)
{
  std::ofstream outputfile(result_file_path_, std::ofstream::out | std::ofstream::app);
  for (size_t i = 0; i < detected_objects.size(); i++)
  {
    // double yaw = tf::getYaw(detected_objects[i].pose.orientation);
    double yaw = detected_objects[i].pose.yaw;

    // KITTI tracking benchmark data format:
    // (frame_number,tracked_id, object type, truncation, occlusion, observation angle, x1,y1,x2,y2, h, w, l, cx, cy,
    // cz, yaw)
    // x1, y1, x2, y2 are for 2D bounding box.
    // h, w, l, are for height, width, length respectively
    // cx, cy, cz are for object centroid

    // Tracking benchmark is based on frame_number, tracked_id,
    // bounding box dimentions and object pose(centroid and orientation) from bird-eye view
    outputfile << std::to_string(frame_count_) << " " << std::to_string(detected_objects[i].id) << " "
               << "Unknown"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1 -1 -1 -1"
               << " " << std::to_string(detected_objects[i].dimensions.x) << " "
               << std::to_string(detected_objects[i].dimensions.y) << " "
               << "-1"
               << " " << std::to_string(detected_objects[i].pose.position.x) << " "
               << std::to_string(detected_objects[i].pose.position.y) << " "
               << "-1"
               << " " << std::to_string(yaw) << "\n";
  }
  frame_count_++;
}

void ImmUkfPda::tracker(const std::vector<BBox>& input,
                        std::vector<BBox>& detected_objects_output)
{
  // 时间戳， 并不一定是 0.1 s 一个 单位 秒
  // double timestamp = input.header.stamp.toSec();
  double timestamp = 0.1;     // 暂时
  // 当前对象是否被别人匹配得到
  std::vector<bool> matching_vec(input.size(), false);

  // 未初始化则执行
  if (!init_)
  {
    // tracks_ ==>  vector<UKF> 初始化
    initTracker(input, timestamp);
    // 丢弃无用测量， 合并对象， 制作输出等
    makeOutput(input, matching_vec, detected_objects_output);
    return;
  }

  // 新旧时间间隔 利用俩帧时间戳
  double dt = (timestamp - timestamp_);
  // 跟新时间帧
  timestamp_ = timestamp;

  // start UKF process
  for (size_t i = 0; i < targets_.size(); i++)
  {
    targets_[i].is_stable_ = false;
    targets_[i].is_static_ = false;
    // 稳定， 静态

    // tracking_num_  表示持续的跟踪帧数
    if (targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }
    // 方差越小估计的越好
    // prevent ukf not to explode
    if (targets_[i].p_merge_.determinant() > prevent_explosion_threshold_ ||
        targets_[i].p_merge_(4, 4) > prevent_explosion_threshold_)
    {
      // 直接判定其死亡
      targets_[i].tracking_num_ = TrackingState::Die;
      continue;
    }

    // 只预测， 并没有用到测量值， 绘制相邻居俩帧的测量运动模型
    // 没有使用到当前测量
    targets_[i].prediction(use_sukf_, has_subscribed_vectormap_, dt);

    std::vector<BBox> object_vec;
    // input 为但前帧测量
    // targets_ 包含对当前帧的预测
    // object_vec 应该为当前帧预测对象所关联的测量对象
    // matching_vec 记录输入对象是否都有与之匹配的预测对象， 可能会有新生的， 也可能会有死亡的
    bool success = probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);
    if (!success)
    {
      continue;
    }
    // 
    targets_[i].update(use_sukf_, detection_probability_, gate_probability_, gating_threshold_, object_vec);
  }
  // end UKF process

  // making new ukf target for no data association objects
  // 为没有被关联到的测量对象新建一个 ukf 对象
  makeNewTargets(timestamp, input, matching_vec);

  // static dynamic classification
  // 动静分离
  staticClassification();

  // making output for visualization
  makeOutput(input, matching_vec, detected_objects_output);

  // remove unnecessary ukf object
  removeUnnecessaryTarget();
}
