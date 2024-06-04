#ifndef THRESHOLD_DISTANCE_CHECK_H
#define THRESHOLD_DISTANCE_CHECK_H
#include <std_msgs/String.h>

#include "collision_check.h"
namespace moveit_servo
{
/**
 * @brief
 * Псевдоним для пары имен линков
 */
using LinkPair = std::pair<std::string, std::string>;
class ThresholdDistanceCollisionCheck : public CollisionCheck
{
  /**
   * @brief
   * Перечисления для определения типа столкновений
   * SELF - для робота и приаттаченного объекта к линку робота Например, меша
   * пробоотборника SCENE - для столкновений робота с неприаттаченными
   * объектами, в богомазе таких объектов на момент написания комментария нет
   */
  enum class CoefType
  {
    SELF,
    SCENE
  };
  /**
   * @brief
   * Структура для хранения результата проверки столкновений
   */
  struct ScaleResult
  {
    double dist; /**< Минимальная дистанция между линками*/
    double scale; /**< Рассчитанный коэффициент замедленния для этой пары линков*/
    LinkPair pair; /**< Пара линков */
    CoefType type; /**< Тип столкновения этой пары линков*/
  };

  /**
   * @brief
   * Структура для хранения границ текущей дистанции(dist) между линками до
   * которой еще можно ехать на полных парах (если dist>warn), надо
   * экспоненциально замедляться (если stop<dist<warn) и двигаться нельзя (если
   * dist<stop)
   */
  struct Threshold
  {
    double warn; /**< Граница выше которой можно ехать с полной скоростью*/
    double stop; /**< Граница ниже которой двигаться нельзя */
    /**
     * @brief
     * Псевдоним для исключения которые могут бросить оба Factory метода если
     * при создании объекта произойдет
     */
    using ThresholdException = std::pair<std::string, CoefType>;

  private:
    // NOTE: Мб бесполезен, так как теперь там линейное снижение а не
    // экспоненциальное
    double scale_ratio; /**< Коэффициент экспоненциального замедления считается
                           как log(0.001)/(stop-warn) */
    double computeScale(double current_dist);

  public:
    static Threshold Factory(double warn, double stop, const CoefType& type);
    /**
     * @brief Проверяет есть ли в структуре value границы для типа type
     * Проверяет только наличие поля self или scene в структуре
     * Не проверяет валидность
     * @param value - Проверяемая структура
     * @param type - тип сцена или самопересечение
     * @return bool - true если есть, иначе false
     */
    static bool haveType(const XmlRpc::XmlRpcValue& value, const CoefType& type);
    static Threshold Factory(const XmlRpc::XmlRpcValue& value, const CoefType& type);
    ScaleResult computeScale(const LinkPair& joint_pair,
                             const std::vector<collision_detection::DistanceResultsData>& dist_data, CoefType type);
  };

  Threshold self_default;  /**< дефолтные допуски для собственных линков
                              (применяется для пары линков для которых нет особой
                              границы в special_self_cases*/
  Threshold scene_default; /**< дефолтные допуски для линков с объектами в сцене
    (кроме приаттаченных) (применяется для пары линков для которых нет особой
    границы в special_scene_cases*/

  /**
   * @brief Читаем с сервера параметров дефолтные границы для сцены и самого
   * робота
   * @param nh - приватный NodeHandle
   * @return bool - false если хотя бы одна из четырех границ не найдена или
   * конфиги не семантически неверны*/
  bool readDefaultParams(ros::NodeHandle& nh);

  /**
   * @brief Читаем с сервера параметрой особые случаи особые случаи храняться
   под адресом special_cases
   * в виде массива следующего вида
   *
    -
     pair: [*имя линка 1*, *имя линка 2*]
     *тут либо self либо scenе*: {stop_mm: 1,warn_mm: 5}
     *еще также можно указать либо scene либо self но не имеет особого смысла,
   так как пара линков обычно пренадлежит к одной из этих групп
    Пример:
    -
     pair: [bogomaz_link_4,tigr_transport_bracket]
     self: {stop_mm: 0.05,warn_mm: 10}
    -
     pair: [bogomaz_link_3,tigr_television]
     self: {stop_mm: 1,warn_mm: 5}

   *
   * @param nh
   */
  void readSpecialCase(ros::NodeHandle& nh);
  /**
   * @brief Структура компоратора которому должно быть все равно на порядок имен
   * в сравниваемых парах шарниров То ест пары {'joint1','joint2'} и
   * {'joint2','joint1'} должны считаться эквивалентными друг другу
   */
  struct OrderLessKeyComparator
  {
    bool operator()(const LinkPair& lhs, const LinkPair& rhs) const;
  };
  std::map<LinkPair, Threshold, OrderLessKeyComparator> special_self_cases;
  std::map<LinkPair, Threshold, OrderLessKeyComparator> special_scene_cases;

  /**
   * @brief Считает коэффициенты снижения скорости для пар линков в dist_map
   *
   * @param dist_map - мапа пары линков
   * @param type - тип к которому относится пара линков, чтобы проверять особые
   * случаи из нужной мапы особых случаев
   * @return std::vector - вектор результатов расчетов коэффициентов снижения
   * скорости
   */
  std::vector<ScaleResult> computeScaleTable(const collision_detection::DistanceMap& dist_map, const CoefType& type);

  /**
   * @brief Вычисляет минимальный коэффициент снижения скорости для результата
   * проверки коллизии в collision_result
   *
   * @param collision_result - результат проверки коллизий для определенного
   * положения манипулятора
   * @return ScaleResult - минимальный scale исходя из данных в collision_result
   */
  ScaleResult getScaleMinimum(const CollisionCheckingResult& collision_result);

  double calcIteration(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future) override;

public:
  ThresholdDistanceCollisionCheck(ros::NodeHandle& nh, const moveit_servo::ServoParameters& parameters,
                                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  friend std::ostream& operator<<(std::ostream& os, const Threshold& th);
  friend std::ostream& operator<<(std::ostream& os, const std::vector<ScaleResult>& computed_coef);
};
std::ostream& operator<<(std::ostream& os, const ThresholdDistanceCollisionCheck::Threshold& th);
std::ostream& operator<<(std::ostream& os,
                         const std::vector<ThresholdDistanceCollisionCheck::ScaleResult>& computed_coef);

}  // namespace moveit_servo
#endif  // THRESHOLD_DISTANCE_CHECK_H
