#include "moveit_servo/threshold_distance_check.h"

#include <xmlrpcpp/XmlRpcException.h>
namespace moveit_servo
{
std::ostream& operator<<(std::ostream& os, const ThresholdDistanceCollisionCheck::Threshold& th)
{
  os << "Warn: " << th.warn * 1000 << "mm Stop: " << th.stop * 1000 << "mm";
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const std::vector<ThresholdDistanceCollisionCheck::ScaleResult>& computed_coef)
{
  for (auto& coef : computed_coef)
  {
    if (coef.scale != 1.0)
    {
      os << "For pair " << std::quoted(coef.pair.first + "-" + coef.pair.second) << ": " << coef.dist << '\n';
    }
  }
  return os;
}
void ThresholdDistanceCollisionCheck::readSpecialCase(ros::NodeHandle& nh)
{
  using namespace XmlRpc;
  XmlRpcValue xml;
  // лямбда для получения пары линков из xmlrpc структуры с проверкой пары по
  // пути
  auto pair_check = [](const XmlRpcValue& element) {
    bool has_pair = element.hasMember("pair");
    if (!has_pair)
      throw std::invalid_argument("Not found key \'pair\' in current element");
    auto pair_xml = element["pair"];
    if (pair_xml.getType() != XmlRpcValue::Type::TypeArray)
    {
      throw std::invalid_argument("Key \'pair\' in current element is not array");
    }
    if (pair_xml.size() != 2)
    {
      throw std::invalid_argument("Key \'pair\' in current element have \'" + std::to_string(pair_xml.size()) +
                                  "\' elements");
    }
    LinkPair pair;
    for (size_t i = 0; i < 2; i++)
    {
      auto pair_element = pair_xml[i];
      if (pair_element.getType() != XmlRpcValue::Type::TypeString)
      {
        throw std::invalid_argument("Element #" + std::to_string(i) + " in key pair is not string");
      }
      if (i == 0)
      {
        pair.first = static_cast<std::string>(pair_element);
      }
      else
      {
        pair.second = static_cast<std::string>(pair_element);
      }
    }
    if (pair.first == pair.second)
    {
      throw std::invalid_argument("Both element in \'pair\' in current element "
                                  "have same name \'" +
                                  pair.first + '\'');
    }
    return pair;
  };
  // чекаем есть ли у нас вообще ~/special_cases на сервер параметров
  if (!nh.getParam("special_cases", xml))
  {
    ROS_INFO_STREAM("No special cases for all link use default threshold\n"
                    "Self: "
                    << self_default
                    << "\n"
                       "Scene: "
                    << scene_default);
    return;
  }
  // Если есть но там не массив xmlrpc структур, то прекращаем анализ
  if (xml.getType() != XmlRpcValue::Type::TypeArray)
  {
    ROS_INFO_STREAM("Special cases is not array for all link use default threshold\n"
                    "Self: "
                    << self_default
                    << "\n"
                       "Scene: "
                    << scene_default);
    return;
  }
  for (int i = 0; i < xml.size(); i++)
  {
    auto& element = xml[i];
    try
    {
      LinkPair pair = pair_check(element);
      for (int j = 0; j < 2; j++)
      {
        auto container_type = j == 0 ? CoefType::SELF : CoefType::SCENE;
        if (!Threshold::haveType(element, container_type))
        {
          continue;
        }
        auto& container = j == 0 ? special_self_cases : special_scene_cases;
        auto str_type = container_type == CoefType::SELF ? "Self" : "Scene";
        if (container.find(pair) == container.end())
        {
          try
          {
            Threshold special_case = Threshold::Factory(element, container_type);
            container[pair] = special_case;
            ROS_WARN_STREAM("Element#" << i << ": " << str_type << " special case for pair "
                                       << std::quoted(pair.first + "-" + pair.second)
                                       << " add special threshold: " << special_case);
          }
          catch (const Threshold::ThresholdException& e)
          {
            ROS_ERROR("Element#%d Catch exception for %s: %s", i, str_type, e.first.c_str());
          }
        }
        else
        {
          ROS_WARN("Element#%d Pair \'%s\'-\'%s\' already in %s_special_case", i, pair.first.c_str(),
                   pair.second.c_str(), str_type);
        }
      }
    }
    catch (const std::invalid_argument& e)
    {
      ROS_WARN("For special case #%d Catch exception \'%s\'. Skip it", i, e.what());
      continue;
    }
  }
}

std::vector<ThresholdDistanceCollisionCheck::ScaleResult>
ThresholdDistanceCollisionCheck::computeScaleTable(const collision_detection::DistanceMap& dist_map,
                                                   const CoefType& type)
{
  std::vector<ScaleResult> result;
  for (auto& dist : dist_map)
  {
    auto& special_map = type == CoefType::SCENE ? special_scene_cases : special_self_cases;
    auto& default_th = type == CoefType::SCENE ? scene_default : self_default;
    auto special_it = special_map.find(dist.first);

    auto& current_th = (special_it == special_map.end()) ? default_th : special_it->second;
    result.push_back(current_th.computeScale(dist.first, dist.second, type));
  }
  return result;
}

ThresholdDistanceCollisionCheck::ScaleResult
ThresholdDistanceCollisionCheck::getScaleMinimum(const CollisionCheckingResult& collision_result)
{
  auto comparator = [](const ScaleResult& a, const ScaleResult& b) { return a.scale < b.scale; };
  auto self_scale_table = computeScaleTable(collision_result.self_distances, CoefType::SELF);
  auto self_minimum = *std::min_element(begin(self_scale_table), end(self_scale_table), comparator);
  if (collision_result.use_scene)
  {
    auto scene_scale_table = computeScaleTable(collision_result.scene_distances, CoefType::SCENE);
    auto scene_minimum = *std::min_element(begin(scene_scale_table), end(scene_scale_table), comparator);
    return std::min(self_minimum, scene_minimum, comparator);
  }
  return self_minimum;
}

double ThresholdDistanceCollisionCheck::calcIteration(const sensor_msgs::JointState&,
                                                      const sensor_msgs::JointState& future)
{
  // Update to the latest current state

  moveit::core::RobotState now_state = getLockedPlanningSceneRO()->getCurrentState();
  moveit::core::RobotState future_state = now_state;
  future_state.setVariableValues(future);

#if THRESHOLD_PARALLEL_COLLISION_CHECK == 1
  auto current =
      std::async(std::launch::async, [this, &now_state]() { return computeCollisionResultForState(now_state); });
  auto next =
      std::async(std::launch::async, [this, &future_state]() { return computeCollisionResultForState(future_state); });
  current.wait();
  next.wait();
  auto current_dist = current.get();
  auto future_dist = next.get();
#else
  auto current_dist = computeCollisionResultForState(now_state);
  auto future_dist = computeCollisionResultForState(future_state);
#endif

  auto current_minimum_pair = getScaleMinimum(current_dist);
  auto future_minimum_pair = getScaleMinimum(future_dist);

  double current_scale_coef = current_minimum_pair.scale;
  double future_scale_coef = future_minimum_pair.scale;

  ROS_DEBUG_STREAM_NAMED("current_collision_distance",
                         "\nCurrent minimum dist: "
                             << current_minimum_pair.dist * 1000 << " mm\nPair: "
                             << std::quoted(current_minimum_pair.pair.first + "-" + current_minimum_pair.pair.second)
                             << "\nScale: " << current_scale_coef);
  ROS_DEBUG_STREAM_NAMED("future_collision_distance",
                         "\nFuture minimum dist: "
                             << future_minimum_pair.dist * 1000 << " mm\nPair: "
                             << std::quoted(future_minimum_pair.pair.first + "-" + future_minimum_pair.pair.second)
                             << "\nScale: " << future_scale_coef);
  if (current_scale_coef < 1.0)
  {
    if (current_scale_coef == 0.0 && future_scale_coef == 0.0)
    {
      ROS_DEBUG_NAMED("threshold_decision", "Both coefs is zero stop moving");
      return 0.0;
    }
    if (current_scale_coef >= future_scale_coef)
    {
      ROS_DEBUG_NAMED("threshold_decision",
                      "Current pose already in warning, but future pose more worth."
                      "Use coef "
                      "from future: %f",
                      future_scale_coef);
      return future_scale_coef;
    }
    else
    {
      ROS_DEBUG_NAMED("threshold_decision", "Current pose already in warning, but future pose good. Use coef "
                                            "1.0");
      return 1.0;
    }
  }
  if (future_scale_coef < 1.0)
  {
    ROS_DEBUG_NAMED("threshold_decision",
                    "Current pose is OK, but future pose already in warning. Use coef from "
                    "future: %f",
                    future_scale_coef);
    return future_scale_coef;
  }
  ROS_DEBUG_NAMED("threshold_decision", "Current pose is OK, Future pose is OK. Use coef 1.0");
  return 1.0;  //Штатное движение
}

ThresholdDistanceCollisionCheck::ThresholdDistanceCollisionCheck(
    ros::NodeHandle& nh, const ServoParameters& parameters,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : CollisionCheck(nh, parameters, planning_scene_monitor)
{
  collision_check_type_ = K_THRESHOLD_DISTANCE;
  if (!readDefaultParams(nh))
    exit(EXIT_FAILURE);

  readSpecialCase(nh);
  if (!use_collision_check)
  {
    ROS_ERROR("WARNING COLLISION CHECKING DISABLED");
  }
  ROS_DEBUG_NAMED("threshold_decision", "Init");
}

bool ThresholdDistanceCollisionCheck::readDefaultParams(ros::NodeHandle& nh)
{
  CoefType type = CoefType::SELF;
  auto getNessary = [&nh, &type](const std::string& param_name) {
    double value;
    if (!nh.getParam(param_name, value))
    {
      throw Threshold::ThresholdException("Not found \'" + nh.resolveName(param_name) + "\' on parameter server", type);
    }
    return convertToROS(value, InputValueType::MILLIMETER);
  };
  try
  {
    self_default = Threshold::Factory(getNessary("self_collision_warning_threshold_mm"),
                                      getNessary("self_collision_stop_threshold_mm"), type);
    type = CoefType::SCENE;
    scene_default = Threshold::Factory(getNessary("scene_collision_warning_threshold_mm"),
                                       getNessary("scene_collision_stop_threshold_mm"), type);
  }
  catch (const Threshold::ThresholdException& e)
  {
    ROS_ERROR("Error in getting default %s parameter: %s", e.second == CoefType::SCENE ? "scene" : "self",
              e.first.c_str());
    return false;
  }
  return true;
}

bool ThresholdDistanceCollisionCheck::OrderLessKeyComparator::operator()(const LinkPair& lhs, const LinkPair& rhs) const
{
  if (lhs.first < lhs.second && rhs.first < rhs.second)
  {
    if (lhs.first == rhs.first)
      return lhs.second < rhs.second;
    return lhs.first < rhs.first;
  }
  else
  {
    if (lhs.first == rhs.second)
      return lhs.second < rhs.first;
    return lhs.first < rhs.second;
  }
}

ThresholdDistanceCollisionCheck::Threshold ThresholdDistanceCollisionCheck::Threshold::Factory(double warn, double stop,
                                                                                               const CoefType& type)
{
  if (warn < 0.0)
    throw ThresholdException("Warn threshold less zero", type);
  if (stop < 0.0)
    throw ThresholdException("Stop threshold less zero", type);
  if (warn <= stop)
    throw ThresholdException("Warn threshold must be greater than stop", type);
  static const double coef = log(0.001);
  Threshold result;
  result.stop = stop;
  result.warn = warn;
  result.scale_ratio = coef / (stop - warn);
  return result;
}

bool ThresholdDistanceCollisionCheck::Threshold::haveType(const XmlRpc::XmlRpcValue& value, const CoefType& type)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    return false;
  }
  std::string type_str = type == CoefType::SCENE ? "scene" : "self";
  return value.hasMember(type_str);
}

ThresholdDistanceCollisionCheck::Threshold
ThresholdDistanceCollisionCheck::Threshold::Factory(const XmlRpc::XmlRpcValue& value, const CoefType& type)
{
  std::array<std::string, 2> param_name = { "warn_mm", "stop_mm" };
  std::array<double, 2> values;
  auto threshold_xml = value[type == CoefType::SCENE ? "scene" : "self"];
  for (size_t i = 0; i < values.size(); i++)
  {
    if (!threshold_xml.hasMember(param_name[i]))
    {
      throw ThresholdException("Current value has not element \'" + param_name[i] + '\'', type);
    }
    auto xml_value = threshold_xml[param_name[i]];
    if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      values[i] = convertToROS(xml_value, InputValueType::MILLIMETER);
    }
    else if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int temp_int = xml_value;
      values[i] = convertToROS(temp_int, InputValueType::MILLIMETER);
    }
    else
    {
      throw ThresholdException("Incorrect type for element \'" + param_name[i] + '\'', type);
    }
  }
  return Factory(values[0], values[1], type);
}

double ThresholdDistanceCollisionCheck::Threshold::computeScale(double current_dist)
{
  if (current_dist > warn)
    return 1.0;
  else if (current_dist <= warn && current_dist > stop)
  {
    double k = 1 / (warn - stop);
    double b = -k * stop;
    return k * current_dist + b;
  }
  else
    return 0.0;
}

ThresholdDistanceCollisionCheck::ScaleResult ThresholdDistanceCollisionCheck::Threshold::computeScale(
    const LinkPair& joint_pair, const std::vector<collision_detection::DistanceResultsData>& dist_data, CoefType type)
{
  ScaleResult result;
  result.pair = joint_pair;
  result.type = type;
  if (dist_data.empty())
  {
    ROS_WARN("Dist data empty return zero");
    result.dist = -1;
    result.scale = 0;
    return result;
  }
  auto min =
      std::min_element(begin(dist_data), end(dist_data),
                       [](const collision_detection::DistanceResultsData& a,
                          const collision_detection::DistanceResultsData& b) { return a.distance < b.distance; });
  result.scale = computeScale(min->distance);
  result.dist = min->distance;
  return result;
}

}  // namespace moveit_servo
