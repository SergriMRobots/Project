#ifndef BUILD_STAMP_H
#define BUILD_STAMP_H
#include <string>
/**
 * @brief
 * Структура получения информации о сборке для включения в бинарный файл
 * Для того чтобы это работало в CMakeLists.txt надо добавить следущие строки:
 * execute_process(
         COMMAND
             git rev-parse HEAD
         OUTPUT_VARIABLE
             GIT_HASH
             OUTPUT_STRIP_TRAILING_WHITESPACE)
   add_definitions("-DGIT_HASH=\"${GIT_HASH}\"")
   add_definitions("-DPROJECT_NAME=\"${PROJECT_NAME}\"")
 */
struct BuildVersion
{
  /**
   * @brief
   * Функция получения хэша коммита на котором был собран бинарный файл
   * @return Строку хэша коммита или "GIT_HASH UNDEFINED" если исходники
   * не под управлением git-a или в CMakeLists.txt нет строк описанных выше
   */
  inline static const std::string GitCommitHash()
  {
#ifndef GIT_HASH
#define GIT_HASH "GIT_HASH UNDEFINED"  // means uninitialized
#endif
    return std::char_traits<char>::length(GIT_HASH) ? GIT_HASH : "GIT_HASH UNDEFINED";
  }

  /**
   * @brief
   * Функция получения имени текущего cmake проекта
   * @return Имя текущего cmake проекта или "PROJECT_NAME UNDEFINED"
   * если исходники не под управлением git-a или в CMakeLists.txt нет строк
   * описанных выше
   */
  inline static const std::string GetProjectName()
  {
#ifndef PROJECT_NAME
#define PROJECT_NAME "PROJECT_NAME UNDEFINED"  // means uninitialized
#endif
    return std::char_traits<char>::length(PROJECT_NAME) ? PROJECT_NAME : "PROJECT_NAME UNDEFINED";
  }
  /**
   * @brief Функция получения строки с полной информацией об условиях сборки
   * бинарники, куда будет включен данный заголовочный файл

   *
   * @return * Строка в виде
   * "Project: *имя проекта* Git hash: *хэш используемого коммита во время
   * сборки* Builded: *дата сборки*"
   */
  inline static const std::string FullInfo()
  {
    return "Project: " + BuildVersion::GetProjectName() + " Git hash: " + BuildVersion::GitCommitHash() +
           " Builded: " + __DATE__ + ' ' + __TIME__;
  }
};
#endif  // BUILD_STAMP_H
