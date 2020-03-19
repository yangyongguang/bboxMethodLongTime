#include "imm_ukf_pda.h"

ImmUkfPda::ImmUkfPda()
  : target_id_(1)  // target_id_ 0
  ,  // assign unique ukf_id_ to each tracking targets
  init_(false),
  frame_count_(0),
  has_subscribed_vectormap_(false)
{
  tracking_frame_ = "world";
  life_time_threshold_ = 8;
  gating_threshold_ = 9.22;
  gate_probability_ = 0.99;
  detection_probability_ = 0.9;
  static_velocity_threshold_ = 0.5;
  static_num_history_threshold_ = 3;
  prevent_explosion_threshold_ = 1000;
  merge_distance_threshold_ = 0.5;
  use_sukf_ = false;

  use_vectormap_ = false;
  lane_direction_chi_threshold_ = 2.71;
  nearest_lane_distance_threshold_ = 1.0;
  vectormap_frame_ = "map";

  is_benchmark_ = false;
  kitti_data_dir_ = "";

  std::ifstream timeStream;
	std::ifstream poseStream;
	string lineStr;
	char ch;
	string timeFileDir = param.kitti_base_velo_dir + "20/info/timestamp.txt";
	std::cout << timeFileDir <<std::endl;
	timeStream.open(timeFileDir, std::ios::in);
	if (timeStream.fail())
	{
		fprintf(stderr, "open timeStream error\n");
    exit(0);
	}
	// while(timeStream >> ch)
	while(timeStream >> lineStr)
	{
		// std::cout << ch << std::endl;
		// std::cout << lineStr << std::endl;
		timestampVec.emplace_back(std::stof(lineStr));
	}
	// for (int idx = 0; idx < timestamp.size(); ++idx)
	// {
	// 	fprintf(stderr, "%f\n", timestamp[idx]);
	// }
	// fprintf(stderr, "num of timestamp : %d\n", timestamp.size());
	timeStream.close();
	// -------------------------------------------------------------
	string poseFileDir = param.kitti_base_velo_dir + "20/info/pose.txt";
	poseStream.open(poseFileDir, std::ios::in);
	if (poseStream.fail())
	{
		fprintf(stderr, "open poseStream error\n");
		exit(1);
	}
	int count = 0;
	while (getline(poseStream, lineStr, '\n'))
	{
		string str;
		int strIdx = 0;
		std::array<float, 3> ps;
		stringstream ss(lineStr);
		// while(getline(ss, str, ' '))
		// {
		// 	ps[strIdx] = std::stof(str);
		// 	strIdx++;
		// }
		ss >> ps[0] >> ps[1] >> ps[2];
		selfCarPose.emplace_back(ps);
	}
  // for (int idx = 0; idx < selfCarPose.size(); ++idx)
  // {
  //   selfCarPose[idx][0] -= selfCarPose[0][0];
  // }
	// --------------------------------------------------------------
	// for (int idx = 0; idx < 100; ++idx)
	// {
	// 	fprintf(stderr, "%f, %f, %f\n", selfCarPose[idx][0], selfCarPose[idx][1], selfCarPose[idx][2]);
	// }
	poseStream.close();
  if (is_benchmark_)
  {
      result_file_path_ = kitti_data_dir_ + "benchmark_results.txt";
      std::remove(result_file_path_.c_str());
  }
}

// // 主函数
// void ImmUkfPda::run()
// {
// //   pub_object_array_ = node_handle_.advertise<std::vector<BBox>>("/detection/objects", 1);
// //   sub_detected_array_ = node_handle_.subscribe("/detection/fusion_tools/objects", 1, &ImmUkfPda::callback, this);

// //   if (use_vectormap_)
// //   {
// //     vmap_.subscribe(private_nh_, vector_map::Category::point |
// //                                  vector_map::Category::NODE  |
// //                                  vector_map::Category::LANE, 1);
// //   }
// }

// 直接使用其回调函数
void ImmUkfPda::callback(const std::vector<BBox>& input, 
      const size_t & currentFrame, 
      vector<Cloud::Ptr> & trackerBBox,
      const int & trackID)
{
  if (DEBUG)
    fprintf(stderr, "ImmUkfPda::callback() start\n");
  trackId_ = trackID;
  std::vector<BBox> transformed_input;
  std::vector<BBox> detected_objects_output;
  // update trans matrix
  updateTransMatrix(currentFrame);
  transformPoseToGlobal(input, transformed_input, currentFrame);
  // 全局 bbox 输入跟踪
  tracker(transformed_input, detected_objects_output, currentFrame);
  // 还原
  transformPoseToLocal(detected_objects_output, currentFrame, trackerBBox);

  if (is_benchmark_)
  {
    dumpResultText(detected_objects_output);
  }
  if (DEBUG)
    fprintf(stderr, "current track ID: %d\n", trackID);
  // fprintf(stderr, "ImmUkfPda::callback() finished\n");
}

void ImmUkfPda::transformPoseToGlobal(const std::vector<BBox>& input,
                                      std::vector<BBox>& transformed_input,
                                      const size_t & currentFrame)
{
  //------------------------------------------------------
  // float theta = selfCarPose[currentFrame][0];
  // float detX = selfCarPose[currentFrame][1];
  // float detY = selfCarPose[currentFrame][2];

  // Eigen::Matrix3f trans;
  // trans << cos(theta),   sin(theta),    0,
  //          -sin(theta),  cos(theta),    0,
  //          detX,         detY,          1;
  // float cos_theta = cos(theta);
  // float sin_theta = sin(theta);
  for (int bboxIdx = 0; bboxIdx < input.size(); ++bboxIdx)
  {
    auto & bbox = input[bboxIdx];
    std::vector<point> tmpPt(4);
    for (size_t idx = 0; idx < 4; ++idx)
    {
      // use pass2
      // tmpPt[idx].x() = bbox[idx].x() * cos_theta - bbox[idx].y() * sin_theta + detX;
      // tmpPt[idx].y() = bbox[idx].x() * sin_theta + bbox[idx].y() * cos_theta + detY;
      tmpPt[idx] = transPointL2G(bbox[idx]);
      // fprintf(stderr, "%f, %f\n", tmpPt[idx].x(), tmpPt[idx].y());
    }
    BBox dd(tmpPt);
    dd.minZ = bbox.minZ;
    dd.maxZ = bbox.maxZ;
    transformed_input.emplace_back(dd);
  }
  // //-------------------------------------------------------------------
  // 可视化所绘制
  // ofstream outFile;
  // string fileName = "/home/yyg/pythonCode/jupyterCode/bboxDataTrans/" + 
  //         std::to_string(currentFrame) + ".txt";
  // outFile.open(fileName, std::ios::out);
  // if (!outFile)
  // {
  //     fprintf(stderr, "open file error \n");
  //     return;
  // }
  // // 保存 bounding box 到 txt 文档， 以供 python 调用
  // for (int idx = 0; idx < transformed_input.size(); ++idx)
  // {
  //     auto & bbox = transformed_input[idx];
  //     for (int pointIdx = 0; pointIdx < 4; ++pointIdx)
  //     {
  //         // use pass1
  //         outFile << bbox[pointIdx].x() << " " << bbox[pointIdx].y() << " ";
  //     }
  //     outFile << "\n";
  // }
  // outFile.close();
  // //-----------------------------------------------------------------
}

void ImmUkfPda::transformPoseToLocal(std::vector<BBox>& detected_objects_output, 
                                      const size_t & currentFrame,
                                      vector<Cloud::Ptr> & trackerBBox)
{
  //------------------------------------------------------
  // float theta = 40.0f / 180 * M_PI;
  // 需要将全局坐标系旋转为正的朝向， 这样后面的计算会容易很多
  float theta = (selfCarPose[currentFrame][0]);  
  // float detX = selfCarPose[currentFrame][1];
  // float detY = selfCarPose[currentFrame][2];

  // Eigen::Matrix3f transG2L, transL2G;
  // float cos_theta = cos(theta);
  // float sin_theta = sin(theta);
  // transL2G << cos_theta, sin_theta, 0,
  //             -sin_theta, cos_theta, 0,
  //             detX, detY, 1;
  // // 求逆矩阵
  // transG2L = transL2G.inverse();

  for (int bboxIdx = 0; bboxIdx < detected_objects_output.size(); ++bboxIdx)
  {
    auto & bbox = detected_objects_output[bboxIdx];
    // 返回可视化 Cloud
    Cloud::Ptr cloudBBox(new Cloud);
    cloudBBox->resize(8);
    cloudBBox->id = bbox.id;  // track id
    cloudBBox->velocity = bbox.velocity.linear.x;
    cloudBBox->acceleration = bbox.acceleration.linear.x;
    cloudBBox->minZ = bbox.minZ;
    cloudBBox->maxZ = bbox.maxZ;
    cloudBBox->yaw = bbox.yaw - theta;
    if (trackId_ == bbox.id)
    {
      fprintf(stderr, "minZ : %f, maxZ : %f\n", bbox.minZ, bbox.maxZ);
      fprintf(stderr, "yaw : %f\n", bbox.yaw / M_PI * 180);
      fprintf(stderr, "self car theta : %f\n", theta / M_PI * 180);
    }
    for (size_t idx = 0; idx < 4; ++idx)
    {
      // use pass2 transform
      // fprintf(stderr, "transform befor (%f, %f)\n", bbox[idx].x(), bbox[idx].y());
      // bug  bbox[idx] 访问的可能是一份复制的值， 并非原 bbox 对象的引用， 因为 operator[] 忘记加 前缀 & 符号
      // bbox[idx].x() = bbox[idx].x() + detX;
      // bbox[idx].y() = bbox[idx].y() + detY;  // rotate
      // // fprintf(stderr, "transform after detX detY (%f, %f)(%f, %f)\n", 
      //           //  bbox[idx].x(), bbox[idx].y(), detX, detY);
      // bbox[idx].x() = bbox[idx].x() * cos_theta - bbox[idx].y() * sin_theta;
      // bbox[idx].y() = bbox[idx].x() * sin_theta + bbox[idx].y() * cos_theta;
      // bug 此处 因为 bbox[idx].x() 在下一行已经提前改变了， 而 在下俩行， 需要的是一个 x 值未改变的相称
      // 所以 需要申请一个临时变量， 待俩者都结束后进行赋值, 参考上俩行的注释错误 提前改变了后面还需要使用的 x 值
      // ----------------------------------------------------------------------------------------
      point tmp = transPointG2L(bbox[idx]);
      // point tmp = bbox[idx];
      // -----------------------------------------------------------------------------------------
      // point tmp;
      // tmp.x() = bbox[idx].x() * transG2L(0, 0) + bbox[idx].y() * transG2L(1, 0) + transG2L(2, 0);
      // tmp.y() = bbox[idx].x() * transG2L(0, 1) + bbox[idx].y() * transG2L(1, 1) + transG2L(2, 1);
      bbox[idx].x() = tmp.x();
      bbox[idx].y() = tmp.y();
      // fprintf(stderr, "rotate after theta (%f, %f)(%f)\n", 
      //            bbox[idx].x(), bbox[idx].y(), theta);
      // 为了给予 _viewer 做可视化
      point bboxPt, bboxPt2;
      bboxPt.x() = tmp.x();
      bboxPt.y() = tmp.y();
      bboxPt.z() = bbox.minZ;

      bboxPt2.x() = tmp.x();
      bboxPt2.y() = tmp.y();
      bboxPt2.z() = bbox.maxZ;
      // cloudBBox->emplace_back(bboxPt);
      (*cloudBBox)[idx] = bboxPt;
      (*cloudBBox)[idx + 4] = bboxPt2;
      // fprintf(stderr, "transform after (%f, %f)\n", (*cloudBBox)[idx].x(), (*cloudBBox)[idx].y());
      // fprintf(stderr, "cloudBBox[%d] (%f, %f)\n", idx,(*cloudBBox)[idx].x(), (*cloudBBox)[idx].y());
    }
    // assert(cloudBBox->size() == 8);
    // // 绘制顶端 bbox
    // auto &outBBox = (*cloudBBox);
    // fprintf(stderr, "----------------------\n");
    // for (int idx = 0; idx < 4; ++idx)
    //   fprintf(stderr, "%f ", outBBox[idx].z());
    // fprintf(stderr, "|<-->|");
    // for (int idx = 4; idx < 8; ++idx)
    //   fprintf(stderr, "%f ", outBBox[idx].z());
    // fprintf(stderr, "\n----------------------\n");
    bbox.updateCenterAndYaw();
    // fprintf(stderr, "after update cloudBBox[idx] (%f, %f)\n", (*cloudBBox)[3].x(), (*cloudBBox)[3].y());
    trackerBBox.push_back(cloudBBox);
  }
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
    if (target_id_ == trackId_)
    {
      fprintf(stderr, "init_meas << %f, %f\n", init_meas(0), init_meas(1));
      fprintf(stderr, "traget_id_ : %d\n", target_id_);
    }
    // 初始化只用到了 x， y 信息， 并没有用到其他信息
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
  //  输入的是 每个预测的值， 关联测量的值
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
    // z 方向
    dd.maxZ = targets_[i].object_.maxZ;
    dd.minZ = targets_[i].object_.minZ;

    // 预测方向
    dd.yaw = tyaw;
    
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

    if (trackId_ == targets_[i].ukf_id_)
    {
      // 打印当前 ukf 信息
      Eigen::VectorXd x_merge = targets_[i].printUKFInfo();
      point toLocal = transPointG2L(point(x_merge(0), x_merge(1), 0.0f));
      fprintf(stderr, "Local state : %f, %f\n", toLocal.x(), toLocal.y());
    }
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
                        std::vector<BBox>& detected_objects_output,
                        const size_t & currentFrame)
{
  // 与上一帧相同， 所以只打印信息， 不更新信息
  bool debugFrame = false;
  // 时间戳， 并不一定是 0.1 s 一个 单位 秒
  // double timestamp = input.header.stamp.toSec();
  if (DEBUG)
  {
    fprintf(stderr, "ImmUkfPda::tracker()\n");
  }
  if (currentFrame_ == currentFrame)
  {
    debugFrame = true;
  }
  float timestamp;
  if (!debugFrame)
    timestamp = timestampVec[currentFrame];
  else
    timestamp = timestamp_;
  // 当前对象是否被别人匹配得到
  std::vector<bool> matching_vec(input.size(), false);

  // 未初始化则执行
  if (!init_)
  {
    // tracks_ ==>  vector<UKF> 初始化
    initTracker(input, timestamp);
    if (DEBUG)
      fprintf(stderr, "initTracker(input, timestamp)\n");
    // 丢弃无用测量， 合并对象， 制作输出等
    makeOutput(input, matching_vec, detected_objects_output);
    if (DEBUG)
      fprintf(stderr, "makeOutput(input, matching_vec, detected_objects_output);\n");
    return;
    }
  double dt;
  if (debugFrame)
    dt = dt_; 
  else
    dt = (timestamp - timestamp_);
  // 新旧时间间隔 利用俩帧时间戳
  // 跟新时间帧
  if (!debugFrame)
    timestamp_ = timestamp;

  // start UKF process
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (trackId_ == targets_[i].ukf_id_)
      targets_[i].debugBool = true;
    targets_[i].is_stable_ = false;
    targets_[i].is_static_ = false;
    // 稳定， 静态

    // tracking_num_  表示持续的跟踪帧数
    // 每个对象初始化结束后，其 tracking_num_ 都会是 1, 
    // 只有经过后面程序判定给予其赋值为 Die
    if (targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }
    // 方差越小估计的越好
    // prevent ukf not to explode
    float pDete = targets_[i].p_merge_.determinant();
    // targets_[i].p_merge_(4, 4) 代表的是角度的方差
    if (trackId_ == targets_[i].ukf_id_)
      fprintf(stderr, "targets_[i].p_merge_.determinant %f\n", pDete);
    if (pDete > prevent_explosion_threshold_ ||
        targets_[i].p_merge_(4, 4) > prevent_explosion_threshold_)
    {
      // 直接判定其死亡
      targets_[i].tracking_num_ = TrackingState::Die;
      continue; // 为下一个目标做准备
    }

    // 只预测， 并没有用到测量值， 绘制相邻居俩帧的测量运动模型
    // 没有使用到当前测量
    targets_[i].prediction(use_sukf_, has_subscribed_vectormap_, dt);

    std::vector<BBox> object_vec;
    // input 为但前帧测量
    // targets_ 包含对当前帧的预测
    // object_vec 应该为当前帧预测对象所关联的测量对象
    // matching_vec 记录输入对象是否都有与之匹配的预测对象， 可能会有新生的， 也可能会有死亡的
    // 关联
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

void ImmUkfPda::updateTransMatrix(const int & currentFrame)
{
  float theta = (selfCarPose[currentFrame][0]);  
  float detX = selfCarPose[currentFrame][1];
  float detY = selfCarPose[currentFrame][2];

  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  transL2G_ << cos_theta, sin_theta, 0,
              -sin_theta, cos_theta, 0,
              detX, detY, 1;
  // 求逆矩阵
  transG2L_ = transL2G_.inverse();
}

point ImmUkfPda::transPointG2L(const point & input)
{
  point tmp;
  tmp.x() = input.x() * transG2L_(0, 0) + input.y() * transG2L_(1, 0) + transG2L_(2, 0);
  tmp.y() = input.x() * transG2L_(0, 1) + input.y() * transG2L_(1, 1) + transG2L_(2, 1);
  return tmp;
}

point ImmUkfPda::transPointL2G(const point & input)
{
  point tmp;
  tmp.x() = input.x() * transL2G_(0, 0) + input.y() * transL2G_(1, 0) + transL2G_(2, 0);
  tmp.y() = input.x() * transL2G_(0, 1) + input.y() * transL2G_(1, 1) + transL2G_(2, 1);
  return tmp;
}