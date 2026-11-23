#ifndef ORIGINCAR_SYSTEM_FOLLOWER_UTILS_HPP_
#define ORIGINCAR_SYSTEM_FOLLOWER_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace origincar_system
{

// 将字符串转成小写，用于 route_direction 等参数的宽松匹配。
inline std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

inline std::string route_direction_name(const std::string & direction)
{
  const auto lower = to_lower(direction);
  if (lower == "clockwise") {
    return "顺时针";
  }
  if (lower == "counterclockwise") {
    return "逆时针";
  }
  if (lower == "auto") {
    return "自动";
  }
  return direction;
}

inline double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace origincar_system

#endif  // ORIGINCAR_SYSTEM_FOLLOWER_UTILS_HPP_
