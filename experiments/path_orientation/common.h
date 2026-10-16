#pragma once

namespace reference_line
{

struct Vec2d
{
  Vec2d() = default;
  Vec2d(double _x, double _y)
    : x(_x)
    , y(_y)
  {}
  Vec2d
  operator+(Vec2d const& other) const
  {
    return {x + other.x, y + other.y};
  }
  Vec2d
  operator/(double const ratio) const
  {
    return {x / ratio, y / ratio};
  }

  double x;
  double y;
};

struct SLPoint
{
  SLPoint() = default;
  SLPoint(double _s, double _l)
    : s(_s)
    , l(_l)
  {}
  double s{0.0};
  double l{0.0};
};

struct InterpolatedIndex
{
public:
  InterpolatedIndex(int id, double offset)
    : id(id)
    , offset(offset)
  {}
  int id = 0;
  double offset = 0.0;
};
} // namespace reference_line
